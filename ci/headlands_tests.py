#!/usr/bin/env python3
import argparse
import json
from math import ceil, floor, atan2, pi, radians, degrees
import matplotlib
from matplotlib.patches import PathPatch
from matplotlib.path import Path
import matplotlib.pyplot as plt
import multiprocessing
import os
from pyproj import Proj
from subprocess import run
from shapely import LineString, MultiPolygon, Polygon, Point
from shapely.ops import unary_union
from time import time


OLD = "old"
NEW = "new"

OPERATING_AREA = "Operating Area"
OBSTACLES = "Obstacles"
TREE_ROWS = "Tree Rows"
RAW_HEADLANDS = "Raw Headlands"
RAW_HEADLANDS_COVERAGE = "Raw Headlands Coverage"
HEADLANDS = "Headlands"
HEADLANDS_COVERAGE = "Headlands Coverage"


def getOutput(cmd):
    data = run(cmd, capture_output=True, shell=True)
    return data.stdout.decode("utf-8"), data.stderr.decode("utf-8")


def getLine(output, substring):
    if output is None:
        return None
    for line in output.splitlines():
        if substring in line:
            return line


def assertIsExteriorOnly(coords, msg):
    assert len(coords) == 1, msg


def getPlotCodes(linear_ring):
    coords = list(linear_ring.coords)
    codes = [Path.LINETO] * len(coords)
    codes[0] = Path.MOVETO
    codes[-1] = Path.CLOSEPOLY
    return coords, codes


def getComplexPlot(complex):
    coords, codes = getPlotCodes(complex.exterior)
    for interior in complex.interiors:
        int_coords, int_codes = getPlotCodes(interior)
        coords = coords + int_coords
        codes = codes + int_codes
    return Path(coords, codes)


# given a list of (x, y) tuples, return a list of all xs and a list of all ys
def getXYLists(coords):
    return list(zip(*coords))[0], list(zip(*coords))[1]


class TestClass:
    def onPick(self, event):
        legend_line = event.artist

        # if the source of the event is not a legend line, do nothing
        if legend_line not in self.map_legend_to_ax:
            return

        # on first click lower alpha, on second click make invisible
        for plot in self.map_legend_to_ax[legend_line]:
            if plot.get_visible() and plot.get_alpha() == 1.0:
                plot.set_alpha(0.6)
                legend_line.set_alpha(0.6)
            elif plot.get_visible() and plot.get_alpha() == 0.6:
                plot.set_visible(False)
                legend_line.set_alpha(0.1)
            elif not plot.get_visible():
                plot.set_visible(True)
                plot.set_alpha(1.0)
                legend_line.set_alpha(1.0)

        self.fig.canvas.draw()

    def getUTMCoords(self, geopoints):
        coords = list()
        for i in range(len(geopoints)):
            utm = self.proj_converter(geopoints[i][0], geopoints[i][1])
            coords.append(utm)
        return coords

    def plotGeojsonFeatureCollection(self, feature_collection):
        if self.args.show_plots:
            for feature in feature_collection["features"]:
                if feature["geometry"]["type"] == "Point":
                    self.ax.scatter(
                        *getXYLists(
                            self.getUTMCoords([feature["geometry"]["coordinates"]])
                        ),
                        color="orangered",
                        zorder=7,
                    )
                elif feature["geometry"]["type"] == "LineString":
                    self.ax.plot(
                        *getXYLists(
                            self.getUTMCoords(feature["geometry"]["coordinates"])
                        ),
                        color="orangered",
                        zorder=7,
                    )
                elif feature["geometry"]["type"] == "Polygon":
                    self.ax.fill(
                        *getXYLists(
                            self.getUTMCoords(feature["geometry"]["coordinates"][0])
                        ),
                        color="orangered",
                        zorder=7,
                    )
                else:
                    raise ValueError(
                        f'Trying to print unhandled feature geometry type: {feature["geometry"]["type"]}'
                    )

    def endTest(self, output, success, msg=None):
        if self.args.print_output:
            output = output + "--\n  input path:\n"
            output = output + f"{self.input_path}\n"

            output = output + "--\n  stderr output:\n"
            output = output + f"{self.stderr}\n"

            output = output + "--\n  stdout output:\n"
            output = output + f"{self.stdout}"

        if success:
            result = "PASSED"
            title_colour = "green"
        else:
            result = f"FAILED - {msg}"
            title_colour = "red"
        output = output + f"| {result}\n"
        output = output + f"| tests run in {time() - self.test_start:.2f} seconds"

        print(output)

        if hasattr(self, "ax") and self.args.show_plots:
            self.ax.set_title(f"{self.ax.get_title()}\n\n{result}", color=title_colour)

            # generate legend in correct order
            handles, labels = self.ax.get_legend_handles_labels()
            handles_ordered = list()
            labels_ordered = list()
            for label in [
                OPERATING_AREA,
                OBSTACLES,
                TREE_ROWS,
                RAW_HEADLANDS,
                RAW_HEADLANDS_COVERAGE,
                HEADLANDS,
                HEADLANDS_COVERAGE,
            ]:
                if label in labels:
                    handles_ordered.append(handles[labels.index(label)])
                    labels_ordered.append(label)
            leg = self.ax.legend(
                handles_ordered,
                labels_ordered,
                fancybox=True,
                shadow=True,
                loc="lower center",
                bbox_to_anchor=(0.5, -0.175),
                ncol=3,
            )

            # position legend on figure
            pos = self.ax.get_position()
            self.ax.set_position(
                [
                    pos.x0 + pos.width * 0.025,
                    pos.y0 + pos.height * 0.05,
                    pos.width * 0.95,
                    pos.height * 0.95,
                ]
            )

            # configure legend to be interactive
            self.map_legend_to_ax = dict()
            for legend_line in leg.get_lines() + leg.get_patches():
                legend_line.set_picker(5)
                self.map_legend_to_ax[legend_line] = self.plots[legend_line.get_label()]
            self.fig.canvas.mpl_connect("pick_event", self.onPick)
            leg.set_draggable(True)

            if not (self.args.show_failed and success):
                plt.show()

    def doTest(self, test_idx, passed_tests, all_tests):
        test = self.test_json["testCases"][test_idx]

        filename = os.path.splitext(os.path.basename(test["inputPayload"]))[0]
        if self.args.test_case is not None and filename != self.args.test_case:
            return
        all_tests.append(filename)

        output = "--------------------\n"

        output = output + f'| Test Case {test_idx}: {test["description"]}\n'

        self.input_path = os.path.abspath(
            os.path.join(self.test_folder, test["inputPayload"])
        )
        if not os.path.isfile(self.input_path):
            output = (
                output + f"Error: Input payload path '{self.input_path}' is not a file."
            )
            return

        # get input data
        input_payload = json.load(open(self.input_path))
        headland_path_buffering_m = input_payload["settings"][
            "biggestHeadlandPathBuffering"
        ]
        self.min_path_turn_radius_m = input_payload["settings"]["minPathTurnRadius"]

        # set up UTM converter
        # regardless of what the projection the farm uses, project into UTM for tests as that is what the robot uses
        zone = test["settings"]["utmZone"]
        hemisphere = test["settings"]["hemisphere"]
        self.proj_converter = Proj(
            f"+proj=utm +zone={zone} +{hemisphere.lower()} +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
        )

        # set up plot
        if self.args.show_plots:
            matplotlib.use("TkAgg")
            self.fig, self.ax = plt.subplots(figsize=(10, 10))
            self.ax.set_title(f'Test {test["description"]}')
            self.ax.set_xlabel(f"Eastings UTM Zone {zone}")
            self.ax.set_ylabel(f"Northings UTM Zone {zone}")
            self.plots = dict()

        # get operating area
        assertIsExteriorOnly(
            input_payload["operatingArea"]["coordinates"],
            "Operating area representation should not have interior borders.",
        )
        operating_area_coords = self.getUTMCoords(
            input_payload["operatingArea"]["coordinates"][0]
        )
        operating_area = Polygon(operating_area_coords)

        if self.args.show_plots:
            operating_area_xs, operating_area_ys = getXYLists(operating_area_coords)

            (operating_area_plot,) = self.ax.fill(
                operating_area_xs,
                operating_area_ys,
                label=OPERATING_AREA,
                color="lightblue",
                alpha=1.0,
            )
            self.plots[OPERATING_AREA] = [operating_area_plot]

            # set plot ranges based on operating area
            range_x = max(operating_area_xs) - min(operating_area_xs)
            range_y = max(operating_area_ys) - min(operating_area_ys)
            centre_x = min(operating_area_xs) + range_x / 2
            centre_y = min(operating_area_ys) + range_y / 2
            range_max = max([range_x, range_y]) * 1.02
            self.ax.set_xlim(
                floor(centre_x - range_max / 2), ceil(centre_x + range_max / 2)
            )
            self.ax.set_ylim(
                floor(centre_y - range_max / 2), ceil(centre_y + range_max / 2)
            )

        # get obstacles
        obstacles = list()
        if self.args.show_plots:
            obstacle_plots = list()

        for obstacle in input_payload["obstacles"]:
            # now that we receive buffered obstacle rings and not raw obstacles, buffered geometries may well have internal rings
            holes = list()
            for i in range(1, len(obstacle["coordinates"])):
                holes.append(self.getUTMCoords(obstacle["coordinates"][i]))
            obstacle = Polygon(
                self.getUTMCoords(obstacle["coordinates"][0]), holes=holes
            )
            obstacles.append(obstacle)

            if self.args.show_plots:
                obstacle_plot = self.ax.add_patch(
                    PathPatch(
                        getComplexPlot(obstacle),
                        label=OBSTACLES,
                        color="black",
                        alpha=1.0,
                        zorder=5,
                    )
                )
                obstacle_plots.append(obstacle_plot)

        if self.args.show_plots:
            self.plots[OBSTACLES] = obstacle_plots

        # get tree rows
        tree_rows = list()
        if "treeRows" in input_payload:
            input_tree_rows = input_payload["treeRows"]
            if self.args.show_plots:
                tree_row_plots = list()

            # for testing, treat adjacent tree rows and tree rows the same
            if "adjacentTreeRows" in input_payload:
                input_tree_rows = input_tree_rows + input_payload["adjacentTreeRows"]

            for tree_row_multi in input_tree_rows:
                tree_row_buffering = tree_row_multi["widths"]["biggest"][
                    "desiredBufferDistance"
                ]
                for tree_single in tree_row_multi["geometry"]["coordinates"]:
                    tree_row_coords = self.getUTMCoords(tree_single)
                    tree_row_line = LineString(tree_row_coords)
                    tree_row_poly = tree_row_line.buffer(tree_row_buffering)
                    tree_rows.append(tree_row_poly)

                    if self.args.show_plots:
                        (tree_rows_plot,) = self.ax.fill(
                            *getXYLists(tree_row_poly.exterior.coords),
                            label=TREE_ROWS,
                            color="green",
                            alpha=1.0,
                            zorder=5,
                        )
                        tree_row_plots.append(tree_rows_plot)

            if self.args.show_plots:
                self.plots[TREE_ROWS] = tree_row_plots

        # generate headlands
        headland_start = time()

        # print("python3 app/headlands.py " + self.input_path + " True True")

        # pass in skip_final_simplification = True -> for testing only, we have the option to not simplify so that we can test the original unsimplified result
        # this is a bit strange since that means we're not testing the actual output, but this is because simplification introduces variance that the turn radius calculation method cannot handle
        # we have not identified a good way to account for the precision loss due to simplification while still correctly detection legitimately problematic turn points
        # so, we just verify that the unsimplified result is good, and trust that the simplified result is also good even if we can't measure it
        # also pass in skip_progress_update = True so we aren't affected by lack of internet
        self.stdout, self.stderr = getOutput(
            "python3 app/headlands.py " + self.input_path + " True True"
        )

        output = (
            output + f"| headlands generated in {time() - headland_start:.2f} seconds\n"
        )

        # begin tests
        self.test_start = time()

        # get headlands
        output_str = getLine(self.stdout, "FeatureCollection")
        if output_str is None:
            num_headlands = 0
        else:
            output_payload = json.loads(output_str)
            headland_payload = output_payload["headlands"]
            headlands = list()
            headland_coverages = list()
            headland_coverages_with_tol = list()
            if self.args.show_plots:
                headland_plots = list()
                headland_coverage_plots = list()

            for headland in headland_payload["features"]:
                assertIsExteriorOnly(
                    headland["geometry"]["coordinates"],
                    "Headlands with interior borders are not valid.",
                )
                headland_coords = self.getUTMCoords(
                    headland["geometry"]["coordinates"][0]
                )
                headland = Polygon(headland_coords)
                headlands.append(headland)

                headland_coverage = LineString(headland_coords).buffer(
                    distance=headland_path_buffering_m
                )
                headland_coverages.append(headland_coverage)
                headland_coverages_with_tol.append(
                    LineString(headland_coords).buffer(
                        distance=headland_path_buffering_m
                        + self.inner_coverage_tolerance_m
                    )
                )

                if self.args.show_plots:
                    (headland_plot,) = self.ax.plot(
                        *getXYLists(headland_coords),
                        "--o",
                        label=HEADLANDS,
                        color="purple",
                        alpha=1.0,
                    )
                    headland_plots.append(headland_plot)
                    headland_coverage_plot = self.ax.add_patch(
                        PathPatch(
                            getComplexPlot(headland_coverage),
                            label=HEADLANDS_COVERAGE,
                            color="orchid",
                            alpha=1.0,
                        )
                    )
                    headland_coverage_plots.append(headland_coverage_plot)

            num_headlands = len(headlands)

            if self.args.show_plots:
                self.plots[HEADLANDS] = headland_plots
                self.plots[HEADLANDS_COVERAGE] = headland_coverage_plots

            total_coverage = unary_union(headland_coverages)

        if "expectNumUnsmoothablePoints" in test["test"]:
            expected_unsmoothable = test["test"]["expectNumUnsmoothablePoints"]
        else:
            expected_unsmoothable = 0

        if "expectUncoverableGaps" in test["test"]:
            expected_uncoverable = test["test"]["expectUncoverableGaps"]
        else:
            expected_uncoverable = 0

        if "exceptNumOtherErrorGeoms" in test["test"]:
            expected_other_error_geoms = test["test"]["exceptNumOtherErrorGeoms"]
        else:
            expected_other_error_geoms = 0

        if not self.args.skip_errors:
            error_payload = json.loads(getLine(self.stderr, "errors"))
            errors = error_payload["errors"]
            if len(errors) != test["test"]["expectNumErrors"]:
                self.endTest(
                    output,
                    success=False,
                    msg=f'Expected {test["test"]["expectNumErrors"]} errors. Got {len(errors)}".',
                )
                return
            else:
                output = output + (
                    f'| - passed number of errors test (expected {test["test"]["expectNumErrors"]})\n'
                )

            # test failures
            unsmoothable_error = None
            coverage_warning = None
            other_error_geoms = 0
            for error in errors:
                if error["errorType"] == "SMOOTHING_FAILURE":
                    unsmoothable_error = error
                elif error["errorType"] == "COVERAGE_WARNING":
                    coverage_warning = error
                else:
                    if "geometry" in error:
                        other_error_geoms = other_error_geoms + len(
                            error["geometry"]["features"]
                        )
                        self.plotGeojsonFeatureCollection(error["geometry"])

            if unsmoothable_error is None:
                unsmoothable = 0
            else:
                # one feature in the geometry collection will be the headland itself, the rest are the unsmoothable points
                unsmoothable = len(unsmoothable_error["geometry"]["features"]) - 1
                self.plotGeojsonFeatureCollection(error["geometry"])

            if coverage_warning is None:
                uncoverable = 0
            else:
                uncoverable = len(coverage_warning["geometry"]["features"])
                self.plotGeojsonFeatureCollection(error["geometry"])

            if unsmoothable != expected_unsmoothable:
                self.endTest(
                    output,
                    success=False,
                    msg=f"Expected {expected_unsmoothable} unsmoothable points. Got {unsmoothable}.",
                )
                return
            else:
                output = (
                    output
                    + f"| - passed number of unsmoothable points test (expected {expected_unsmoothable})\n"
                )

            if uncoverable != expected_uncoverable:
                self.endTest(
                    output,
                    success=False,
                    msg=f"Expected {expected_uncoverable} uncoverable gaps. Got {uncoverable}.",
                )
                return
            else:
                output = (
                    output
                    + f"| - passed number of uncoverable gaps test (expected {expected_uncoverable})\n"
                )

            if other_error_geoms != expected_other_error_geoms:
                self.endTest(
                    output,
                    success=False,
                    msg=f"Expected {expected_other_error_geoms} other error geometries. Got {other_error_geoms}.",
                )
                return
            else:
                output = (
                    output
                    + f"| - passed number of other error geometries test (expected {expected_other_error_geoms})\n"
                )

            # test expected number of headlands
            if test["test"]["expectNumHeadlands"] != num_headlands:
                self.endTest(
                    output,
                    success=False,
                    msg=f'Excepted {test["test"]["expectNumHeadlands"]} headlands to be generated. Got {num_headlands}.',
                )
                return
            else:
                output = (
                    output
                    + f'| - passed number of headlands test (requested {input_payload["settings"]["numHeadlands"]}, expected {test["test"]["expectNumHeadlands"]})\n'
                )

        if num_headlands > 0:
            # test boundaries
            if not self.args.skip_boundaries:
                operating_area_plus_tol = operating_area.buffer(
                    self.boundary_tolerance_m
                )
                if not operating_area_plus_tol.contains(total_coverage):
                    if self.args.show_plots:
                        failed = total_coverage.difference(
                            operating_area_plus_tol.intersection(total_coverage)
                        )
                        if isinstance(failed, MultiPolygon):
                            for segment in failed.geoms:
                                self.ax.add_patch(
                                    PathPatch(
                                        getComplexPlot(segment),
                                        color="red",
                                        zorder=7,
                                    )
                                )
                        else:
                            self.ax.add_patch(
                                PathPatch(
                                    getComplexPlot(failed),
                                    color="red",
                                    zorder=7,
                                )
                            )
                    self.endTest(
                        output,
                        success=False,
                        msg=f"Headland border exceeds operating area boundaries by more than {self.boundary_tolerance_m}m tolerance.",
                    )
                    return
                else:
                    output = (
                        output
                        + f"| - passed border boundaries test ({self.boundary_tolerance_m}m tolerance)\n"
                    )

                failed_obstacles = list()
                for obstacle in obstacles:
                    fail = False
                    for headland in headlands:
                        if obstacle.buffer(-self.boundary_tolerance_m).intersects(
                            headland.exterior
                        ):
                            fail = True
                    if fail:
                        failed_obstacles.append(obstacle)
                        if self.args.show_plots:
                            self.ax.plot(*obstacle.exterior.xy, color="red", zorder=7)
                if len(failed_obstacles) > 0:
                    self.endTest(
                        output,
                        success=False,
                        msg=f"Headland crosses boundaries of {len(failed_obstacles)} obstacles by more than {self.boundary_tolerance_m}m tolerance.",
                    )
                    return
                else:
                    output = (
                        output
                        + f"| - passed obstacle boundaries test ({self.boundary_tolerance_m}m tolerance)\n"
                    )

                failed_tree_rows = list()
                for tree_row in tree_rows:
                    fail = False
                    for headland in headlands:
                        if tree_row.buffer(-self.boundary_tolerance_m).intersects(
                            headland.exterior
                        ):
                            fail = True
                    if fail:
                        failed_tree_rows.append(tree_row)
                        if self.args.show_plots:
                            self.ax.plot(*tree_row.exterior.xy, color="red", zorder=7)
                if len(failed_tree_rows) > 0:
                    self.endTest(
                        output,
                        success=False,
                        msg=f"Headland crosses boundaries of {len(failed_tree_rows)} tree rows by more than {self.boundary_tolerance_m}m tolerance.",
                    )
                    return
                else:
                    output = (
                        output
                        + f"| - passed tree row boundaries test ({self.boundary_tolerance_m}m tolerance)\n"
                    )

            # test border coverage
            if not self.args.skip_coverage:
                reachable_operating_area = operating_area
                coverage_minus_obstacles = total_coverage

                # obstacles or tree rows may affect the possible coverage level
                for obstacle_tree_row in obstacles + tree_rows:
                    # subtract area from reachable area
                    reachable_operating_area = reachable_operating_area.difference(
                        obstacle_tree_row
                    )

                    # also subtract area from the coverage area, since the geometries we have already include the correct coverage distance around each raw obstacle or tree row
                    coverage_minus_obstacles = coverage_minus_obstacles.difference(
                        obstacle_tree_row
                    )

                # calculate the border coverage based on the exterior ring only, since internal holes are irrelevant
                # if we have created a multipolygon, sum the areas
                if isinstance(coverage_minus_obstacles, Polygon):
                    coverage_area = Polygon(
                        coverage_minus_obstacles.exterior.coords
                    ).area
                elif isinstance(coverage_minus_obstacles, MultiPolygon):
                    coverage_area = 0
                    for poly in coverage_minus_obstacles.geoms:
                        coverage_area = (
                            coverage_area + Polygon(poly.exterior.coords).area
                        )

                if isinstance(reachable_operating_area, Polygon):
                    reachable_area = Polygon(
                        reachable_operating_area.exterior.coords
                    ).area
                elif isinstance(reachable_operating_area, MultiPolygon):
                    reachable_area = 0
                    for poly in reachable_operating_area.geoms:
                        reachable_area = (
                            reachable_area + Polygon(poly.exterior.coords).area
                        )

                coverage_percent = coverage_area / reachable_area * 100
                if coverage_percent < test["test"]["borderCoverageTolerancePercent"]:
                    self.endTest(
                        output,
                        success=False,
                        msg=f'Headland border coverage {coverage_percent:.3f}% is less than tolerance of {test["test"]["borderCoverageTolerancePercent"]}%.',
                    )
                    return
                else:
                    output = (
                        output
                        + f'| - passed border coverage test with {coverage_percent:.3f}% ({test["test"]["borderCoverageTolerancePercent"]}% tolerance)\n'
                    )

                # test gaps between headlands
                total_coverage_with_tol = unary_union(headland_coverages_with_tol)
                failed = 0
                for gap in total_coverage_with_tol.interiors:
                    gap = Polygon(gap.coords)
                    if not headlands[-1].contains(gap):
                        for obstacle in obstacles:
                            if gap.intersects(obstacle):
                                # if the obstacle we recieve in the payload is the edge of the ring that the robot can drive around it, we can allow a band of missed coverage on the outside
                                gap = gap.difference(
                                    obstacle.buffer(headland_path_buffering_m)
                                )
                        if not gap.is_empty:
                            if self.args.show_plots:
                                if isinstance(gap, Polygon):
                                    gap = MultiPolygon([gap])
                                for segment in gap.geoms:
                                    self.ax.fill(
                                        *segment.exterior.coords.xy,
                                        color="red",
                                        zorder=7,
                                    )
                            failed = failed + 1
                if failed != expected_uncoverable:
                    self.endTest(
                        output,
                        success=False,
                        msg=f"Coverage between headlands contains {failed} gaps larger than tolerance of {self.inner_coverage_tolerance_m}m (expected {expected_uncoverable}).",
                    )
                    return
                else:
                    output = (
                        output
                        + f"| - passed gap coverage test (expected {expected_uncoverable}, {self.inner_coverage_tolerance_m}m tolerance)\n"
                    )

            # test turn radius
            # NOTE: this uses the same logic to determine turn radius that the new headland generator algorithm uses to verify itself
            # however, it is deliberately copied here instead of calling the same code, to decouple this test from changes to the algorithm
            if not self.args.skip_turn:
                failed = 0
                highest_turn_angle = 0
                failed_radius_xs = list()
                failed_radius_ys = list()

                # for the given sampling distance, there is a max angle through the sampled points that can be calculated from the min turn radius, such that if angle_between is higher than this max then the turn is too sharp
                # the relationship between the max angle and the sample distance is taken by imagining the section of path as an arc of a circle, such that sample distance = arc length = min turn radius * central angle (in rads)
                # thus, max turn angle = smoothing_sampling_distance/min_turn_radius_m rads
                turn_limit = (
                    self.turn_radius_sampling_distance_m / self.min_path_turn_radius_m
                )
                turn_limit_with_tol = turn_limit + radians(
                    self.turn_radius_tolerance_deg
                )

                for headland in headlands:
                    # headland should be ring with last coord same as first coord
                    assert headland.exterior.coords[-1] == headland.exterior.coords[0]

                    # no need to loop until last coord since it is the same as first
                    for i in range(len(headland.exterior.coords) - 1):
                        # reorder headland cords to start at the current point index to make interpolating distance easier
                        shuffled_headland = LineString(
                            headland.exterior.coords[i:-1]
                            + headland.exterior.coords[:i]
                            + [headland.exterior.coords[i]]
                        )

                        # take 2 points link_length metres on either side of specific idx
                        p1 = shuffled_headland.interpolate(
                            -self.turn_radius_sampling_distance_m
                        )
                        p2 = shuffled_headland.interpolate(
                            self.turn_radius_sampling_distance_m
                        )

                        # get bearings between current point and the points on either side
                        bearing_1 = atan2(
                            p1.coords[0][1] - headland.exterior.coords[i][1],
                            p1.coords[0][0] - headland.exterior.coords[i][0],
                        )
                        bearing_2 = atan2(
                            p2.coords[0][1] - headland.exterior.coords[i][1],
                            p2.coords[0][0] - headland.exterior.coords[i][0],
                        )
                        angle_between = abs(pi - abs(bearing_2 - bearing_1))

                        if angle_between > highest_turn_angle:
                            highest_turn_angle = angle_between

                        if angle_between > turn_limit_with_tol:
                            failed = failed + 1
                            failed_radius_xs.append(headland.exterior.coords[i][0])
                            failed_radius_ys.append(headland.exterior.coords[i][1])

                if failed > expected_unsmoothable:
                    if self.args.show_plots:
                        self.ax.scatter(
                            failed_radius_xs, failed_radius_ys, color="red", zorder=7
                        )
                    self.endTest(
                        output,
                        success=False,
                        msg=f"Headland contains {failed} points with turn angle up to {degrees(highest_turn_angle):.3f}°, over the limit of {degrees(turn_limit):.3f}° ({self.turn_radius_tolerance_deg}° tolerance).",
                    )
                    return
                else:
                    output = (
                        output
                        + f"| - passed turn radius test with highest turn angle of {degrees(highest_turn_angle):.3f}° ({degrees(turn_limit):.3f}° limit, {self.turn_radius_tolerance_deg}° tolerance)\n"
                    )

        passed_tests.append(filename)
        self.endTest(output, success=True)

    def test(self):
        parser = argparse.ArgumentParser(description="Run all test cases")
        parser.add_argument(
            "--version",
            type=str,
            default="Release",
            help="Test 'Debug' or 'Release' version.",
        )
        parser.add_argument(
            "--test-file",
            type=str,
            help="Name of test file (excluding extension), 'tests' by default.",
        )
        parser.add_argument(
            "--test-case",
            type=str,
            help="Specify single test file to run.",
        )
        parser.add_argument(
            "--sequential",
            action="store_true",
            help="Run tests in seqeunce instead of in parallel for more accuate speed benchmarking.",
        )
        parser.add_argument(
            "--skip-errors",
            action="store_true",
            help="Do not run error message or failed points tests.",
        )
        parser.add_argument(
            "--skip-boundaries",
            action="store_true",
            help="Do not run boundaries tests.",
        )
        parser.add_argument(
            "--skip-coverage",
            action="store_true",
            help="Do not run coverage tests.",
        )
        parser.add_argument(
            "--skip-turn",
            action="store_true",
            help="Do not run turn radius tests tests.",
        )
        parser.add_argument(
            "--print-output",
            action="store_true",
            help="Print test execution output.",
        )
        parser.add_argument(
            "--show-plots",
            action="store_true",
            help="Display test case data in plot.",
        )
        parser.add_argument(
            "--show-failed",
            action="store_true",
            help="Display plots for failed cases only.",
        )
        self.args = parser.parse_args()

        # toggle show_plots also if we are showing only failed plots
        if self.args.show_failed:
            self.args.show_plots = True

        self.test_folder = "ci/test_cases"

        all_tests_start = time()

        if self.args.test_file:
            test_file = self.args.test_file
        else:
            test_file = "tests"

        self.test_json = json.load(
            open(os.path.join(self.test_folder, f"{test_file}.json"))
        )

        # get common test parameters
        self.boundary_tolerance_m = self.test_json["testParameters"][
            "boundaryToleranceM"
        ]
        self.inner_coverage_tolerance_m = self.test_json["testParameters"][
            "innerCoverageToleranceM"
        ]
        self.turn_radius_sampling_distance_m = self.test_json["testParameters"][
            "turnRadiusSamplingDistanceM"
        ]
        self.turn_radius_tolerance_deg = self.test_json["testParameters"][
            "turnRadiusToleranceDeg"
        ]

        test_range = range(len(self.test_json["testCases"]))

        if self.args.sequential:
            self.passed_tests = list()
            self.all_tests = list()
            for test_idx in test_range:
                self.doTest(test_idx, self.passed_tests, self.all_tests)
        else:
            manager = multiprocessing.Manager()
            self.passed_tests = manager.list()
            self.all_tests = manager.list()
            processes = list()
            for test_idx in test_range:
                processes.append(
                    multiprocessing.Process(
                        target=self.doTest,
                        args=(test_idx, self.passed_tests, self.all_tests),
                    )
                )
            for process in processes:
                process.start()
            for process in processes:
                process.join()

        print("--------------------")
        if len(self.all_tests) == 0:
            print(f"Test Case '{self.args.test_case}' not found")
        elif len(self.passed_tests) == len(self.all_tests):
            print("ALL PASSED")
        else:
            failed_str = "FAILED CASES:"
            for test in self.all_tests:
                if test not in self.passed_tests:
                    failed_str = failed_str + f" {test},"
            print(failed_str[:-1])
        print(f"all tests run in {time() - all_tests_start:.2f} seconds")


if __name__ == "__main__":
    test_class = TestClass()
    test_class.test()
