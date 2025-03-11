#!/usr/bin/env python3
import json
from enum import Enum
from math import atan2, ceil, cos, degrees
from math import dist as dist_2d
from math import floor, pi, radians, sin
from sys import argv, stderr
from time import time

import boto3
import dubins
import requests
from botocore.exceptions import ClientError
from pyproj import Proj
from shapely import LineString, MultiPolygon, Point, Polygon, to_geojson
from shapely.geometry.polygon import orient
from shapely.ops import linemerge, nearest_points, split, unary_union

"""
General Headland Config
"""


# value to be subtracted from configured coverage distance when buffering or projecting headlands, to allow headlands to overlap
# while buffering to the exact coverage distance will result in some small gaps, overlap should be able to stay at 0 as these imprecisions are handled by tolerance factors
# however, overlap value is made available to be changed for testing purposes
headland_overlap_m = 0

# applied in opposite direction to headland_overlap_m, distance that can be left uncovered when projecting coverage, trading coverage for reduced amount of turns in the path
# should stay at 0 because full coverage is generally desired, however is made available to be changed for testing purposes
allow_missed_coverage_m = 0


"""
General Geometric Config
"""


# nominal distance to buffer a 1D LineString by in order to transform it into a valid 2D Polygon
polygonise_dist_m = 0.001

# tolerance allowed in all boundary tests to account for mathematical imprecision of geometric functions
geometric_precision_error_m = 0.02

# tolerance distance to simplify headlands to during processing
# NOTE: smoothing_turn_angle_tolerance_deg and processing_simplification_precision_m, the 2 safety_dist_ms must be tuned together such that a path generated to be correct does indeed test as correct after simplification
# otherwise we will end up in an infinite loop re-introducing coverage gaps by re-smoothing out projected segments, and repeated generating "smoothed" segments that still test to be untraversable
processing_simplification_precision_m = 0.015

# an extra simplification could possibly be done at a different tolerance after processing, before returning, to reduce output data size
final_simplification_precision_m = 0.05

# even if it can be simplified, to avoid distortions on long straight lines caused by differing projection systems, limit the min distance between points on the final headland output
max_point_interval_m = 100


"""
General Path Generation Config
"""


# point precision distance to generate dubins paths at, and to specify for buffering if do_smooth_by_buffering is enabled (which it currently isn't)
# possibly, if do_smooth_by_buffering is enabled and tuned properly, this should be split to be a different value for dubins and for buffering
path_generation_precision_m = 0.5

# when generating a dubins path between 3 points, this is the max multiplier that the projected path length can be of the straight line distance between the waypoints
# having this higher will produce paths with more turns and more severe curvature
path_max_extension_factor = 1.1


"""
MultiPolygon Handling Config
"""

# when iteratively buffering in a shape with bottlenecks in order to identify the best path between resulting subpolygons, this is the starting buffering step
connect_multipolyon_step_default_m = 3

# if the step above has to be reduced because the default buffers too far to find a traversable connection between subpolygons, this is how much to reduce it by each try
connect_multipolyon_step_precision_m = 0.5


"""
Smoothing Config
"""


# this is the distance at which to sample a line in order to calculate its turn radius, which must be strict enough to detect all problematic turns but lenient enough to take into account precision loss due to simplification
# a sampling distance too small and a perfect circle created by buffering or dubins to the correct turn radius, once simplified to a reasonable precision, will fail to be tested as traversable (even with tolerances)
# because this values defines the interpolated distance along the path between sample points, and not the straight-line distance, it doesn't correspond directly to the robot's link length, although it shoudl be roughly similar
# link length does vary between robots, so we could determine sampling distance dynamically from an accurate link length value from the payload, but since it's an approximately-chosen value anyway it's okay to be hard-coded for now
# a common link length is 1.6m, so this value was determined experimentally based roughly on that, but tuned until we saw reasonable-looking results in combination with the tolerance and simplification config values
smoothing_sampling_dist_m = 1.65

# max angle magnitude (degrees) allowed above the limit when testing path traversability
# NOTE: smoothing_turn_angle_tolerance_deg and processing_simplification_precision_m, the 2 safety_dist_ms must be tuned together such that a path generated to be correct does indeed test as correct after simplification
# otherwise we will end up in an infinite loop re-introducing coverage gaps by re-smoothing out projected segments, and repeated generating "smoothed" segments that still test to be untraversable
smoothing_turn_angle_tolerance_deg = 0.75

# extra distance (m) added to the min turning radius value used in dubins generation, to allow for safety and correct for the loss of precision due to simplification
# NOTE: smoothing_turn_angle_tolerance_deg and processing_simplification_precision_m, the 2 safety_dist_ms must be tuned together such that a path generated to be correct does indeed test as correct after simplification
# otherwise we will end up in an infinite loop re-introducing coverage gaps by re-smoothing out projected segments, and repeated generating "smoothed" segments that still test to be untraversable
dubins_radius_safety_dist_m = 0.2

# a way of speeding up is to first attempt a simple smoothing by buffering out and then in by the desired turn radius, and only perform the full algorithm if that isn't possible due to buffering creating a multipolygon
# this can attempted in both combination of directions -> out then in smooths concave turns, in then out smooths convex turns
# currently disabled because, though it should theoretically produce traversable paths, buffering is somehow producing results that are determined to be untraversable at the configured smoothing_sampling_dist_m
# this breaks assumptions that paths corrected once will continue to test as traversable, and could lead to unexpected behaviour of the algorithm, as well as being counterproductive as an attempt to increase speed
# this process could possibly be enabled again if it is investigated, tested, and refined more thoroughly
do_smooth_by_buffering = False

# not in use if do_smooth_by_buffering is false
# extra distance (m) added to the min turning radius value used when buffering to smooth, to allow for safety and correct for the loss of precision due to simplification
# NOTE 1: smoothing_turn_angle_tolerance_deg and processing_simplification_precision_m, the 2 safety_dist_ms must be tuned together such that a path generated to be correct does indeed test as correct after simplification
# otherwise we will end up in an infinite loop re-introducing coverage gaps by re-smoothing out projected segments, and repeated generating "smoothed" segments that still test to be untraversable
# NOTE 2: possibly finding the correct relationship with dubins_radius_safety_dist_m, as well as with path_generation_precision_m, for the configured smoothing_sampling_dist_m, is the way to make smoothing by buffering viable?
buffer_smoothing_safety_dist_m = 0.1

# the following configs pertain to priority iterator logic, see for a full explanation: file://./../docs/priority_iterators.md

# lowest splice distance to try when smoothing
min_smoothing_splice_dist_m = 1

# highest splice distance to try when smoothing
max_smoothing_splice_dist_m = 150

# step at which to move up when trying distance pairs
smoothing_splice_dist_step_m = 2

# number of intervals between the distance pairs to try, i.e. if num_intervals = 10 then first an interval of 0 will be tried, then an interval of 1/10 * max distance, then 2/10 * max distance, etc.
smoothing_splice_dist_num_intervals = 10

# number of sections that the intervals, as configured above, will be further broken into for priority order -> if interval_sections >= num_intervals then this value will have no effect
# e.g. if sections = 4 then first only pairs up to 1/4 of the total range apart will be tried, then pairs up to 1/2 of the total range apart, then 3/4 of the range apart
smoothing_splice_dist_interval_sections = 4

# because the distances tried only have integer precision, define distance under which intervals ranges are no longer broken into sections because they would be too small
smoothing_splice_dist_interval_sectioning_floor_m = 10

# small distance to shift the centre point when doing 3 point smoothing by projection
# this is to shift it just slightly away from the boundary, because paths generated right to the edge of what's legal are very likely to fail checks due to a slight deviation
smoothing_projection_adjustment_dist_m = 0.01

"""
Filling Coverage Config
"""


# max width of a gap that can be disregarded when testing for gaps in coverage
# setting this too low can not only cause speed reduction, but may potentially send fill_coverage into an infinite loop if it bounces between multiple gaps it can't all fill
# NOTE: this value is only tested as the width in the dimension perpendicular to the headland, and does not correlate to the length of the gap parallel to the headland
gap_tolerance_m = 0.1

# because coverage is projected to a single furthest point that's determined after subtracting tolerances, allowing extra overlap gives a better chance of actually covering a gap fully
coverage_projection_extra_overlap_m = 0.05

# the following configs pertain to priority iterator logic, see for a full explanation: file://./../docs/priority_iterators.md

# lowest splice distance to try when filling coverage
min_coverage_splice_dist_m = 3

# highest splice distance to try when filling coverage -> though rounding will make any number viable, will only make regular incrementations if max = (muliple of step) + min
max_coverage_splice_dist_m = 107

# step at which to move up when trying distance pairs
coverage_splice_dist_step_m = 4

# number of sections that the total distance range is broken into, such that only distances within a single range will be tried against each other
coverage_splice_distance_sections = 2

"""
Timeouts
"""
# make_traversable will loop until it successfully makes all points either successfully smoothed or specifically marked as unsmoothable, which should always happen when configured properly
# however, if given a poor combination of configs, it's possible make_traversable could end up continually trying but failing to smooth points and loop infinitely
# this timeout guards against infinitely looping by specifying a max number of attempted smoothing iterations allowed before the function excepts
traversable_timeout_loops = 200

# in addition to the iterations within make_traversable, the entire algorithm has a known limitation which is handled externally by repeatedly calling the entire function
# for the sake of performance, each unique point is only ever tested once for traversability, and any previously tested point is skipped in future checks
# however, a point's traversability depends on the segments around it, which can be altered if the point itself isn't altered, so a previously confirmed traversable point may not remain traversable
# in practice, this appears sufficient to make the entire path traversable in the vast majority of cases, so it is still slow and unnecessary to continually retest the same points
# it is faster to simply call the entire function again after the first sweep, because then every point that previously passed only has to get re-checked once
# this calling needs its own timeout to prevent infinite looping, it could end up bouncing back and forth making alterations that override each other
# at least 2 is the minimum possible required, and it appears this is enough in most cases -> 1 to do most of the work and just 1 extra to pick up the very rare stragglers
# when more iterations are needed, it's generally only after the 2nd headland, since coverage projections in the 2nd headland form weirder shapes to attempt to fill
# in actual usage, more than 2 headlands are rarely actually requested, but they are officially supported -> the maximum number of iterations ever hit by any of the 3 headlands test cases is 4
# since this is a maximum upper limit timeout guard, and in most cases the loop will exit earlier due to having nothing left to process, we may as well have it slightly higher
max_make_traversable_iterations = 5

# fill_coverage will loop until it successfully makes all coverage gaps either successfully covered or specifically marked as uncoverable, which should always happen when configured properly
# however, if given a poor combination of configs, it's possible fill_coverage could end up continually trying but failing to fill gaps and loop infinitely
# this timeout guards against infinitely looping by specifying a max number of attempted smoothing iterations allowed before the function excepts
coverage_timeout_loops = 200


"""
Error Handling
"""


# enum for different types of errors, which should be handled differently
class ErrorType(Enum):
    # either payload missing elements or an unsupported mismatch of settings, pass onto front end
    BAD_INPUT_DATA = 0

    # unexpected logic error in algorithm, should not be passed onto front end
    ALGORITHM_ERROR = 1

    # input paddock is shaped such that total requested number of headlands cannot be generated, pass onto front end possibly along with error geometries
    # headlands that were successfully generated will still be returned
    INNER_HEADLAND_FAILURE = 2

    # input paddock has an area too narrow for the robot to fit through due to either its shape or an obstacle, pass onto front end possibly along with error geometries
    # if headlands can still be succesfully generated for part of the subfield, they willl still be returned
    MALFORMED_OPERATING_AREA = 3

    # make_traversable algorithm failed to find a way to correct one or more points, pass onto front end along with error geometries containing both failed points and the unsmoothed headland for reference
    # if this happens for an inner headland, the other successfully generated headlands will still be returned
    SMOOTHING_FAILURE = 4

    # fill_coverage algorithm failed to find a way to fill one or more gaps, pass onto front end along with error geometries containing the uncovered gaps
    # because this may be the best we can do for now, and there are no safety risks, this is only a warning not a full error so all headlands are still succesfully returned as-is
    COVERAGE_WARNING = 5


# data struct for errors
class Error:
    def __init__(self, error_type, message, geometry=None):
        self.error_type = error_type
        self.message = message
        self.geometry = geometry  # optional

    def as_dict(self):
        error_dict = dict()
        error_dict["errorType"] = self.error_type.name
        error_dict["message"] = self.message
        if self.geometry is not None:  # optional
            error_dict["geometry"] = self.geometry
        return error_dict


# exception for headland generation failure that should be passed on to the front end, along with possible extra info
class GenerationError(Exception):
    def __init__(self, error_type, message, geometry=None):
        super().__init__(message)
        self.error = Error(error_type, message, geometry)


# exception for internal logic errors that should not, if algorithm is functioning as expected, ever be raised
# if raised, they should not be passed on to the front end as they are not meaningful to the end user
class AlgorithmError(Exception):
    def __init__(self, message, geometry=None):
        super().__init__(message)
        self.error = Error(ErrorType.ALGORITHM_ERROR, message, geometry)


# function to transform error objects into a list of dictionaries in the format the lambda is expecting output
# all places constructing returns should use this for streamlining to make sure the structure is always correct
def make_error_list_return(errors):
    if isinstance(errors, Exception):
        errors = [errors.error]

    errors_dict = dict()
    error_list = list()
    for error in errors:
        error_list.append(error.as_dict())
    errors_dict["errors"] = error_list

    return errors_dict


"""
Obstacle / Tree Row Tag 
"""


class ObstacleTag(Enum):
    OBSTACLE = 0
    TREE_ROW = 1


"""
Basic Utility Functions
"""


def assert_is_exterior_only(coords, msg):
    assert len(coords) == 1, msg


def swap(a, b):
    return b, a


# check if equal with tolerance to handle floating point error
def approx_equals(a, b, tol=0.001):
    if abs(a - b) <= tol:
        return True
    else:
        return False


# check if equal with tolerance to handle floating point error
def point_approx_equals(p1, p2, tol=0.001):
    return approx_equals(p1[0], p2[0], tol) and approx_equals(p1[1], p2[1], tol)


def to_line_string(polygon):
    return LineString(polygon.exterior.coords).simplify(
        processing_simplification_precision_m
    )


def normalise_angle(angle):
    if angle < -pi:
        angle = angle + 2 * pi
    elif angle > pi:
        angle = angle - 2 * pi
    return angle


def get_angle(p1, p2):
    try:
        angle = atan2((p2[1] - p1[1]), (p2[0] - p1[0]))
    except ZeroDivisionError:
        angle = pi / 2 if (p2[1] - p1[1]) > 0 else -pi / 2
    return normalise_angle(angle)


def get_segment_between(lines, p1, p2):
    for line in lines:
        if line.intersects(p1) and line.intersects(p2):
            return line


# make a polygon connection between two polygons
# first create a LineString, then buffer by a nominal amount to transform into a valid polygon
def make_connection_between(polygon1, polygon2, quad_segs):
    connection_points = nearest_points(polygon1, polygon2)
    return LineString([connection_points[0], connection_points[1]]).buffer(
        polygonise_dist_m, quad_segs=quad_segs
    )


"""
Polygon Functions
"""


# add interpolated points to a polygon such as the distance between any 2 consecutive points is no greater than the specified max interval
def interpolate_polygon_at_interval(polygon, max_interval):
    # get polygon exterior as LineString for interpolation functios
    line = LineString(polygon.exterior.coords)

    # list of new points including both original and interpolated points
    new_points = list()

    # add first original point to new list, and begin with considering the second original point
    new_points.append(line.coords[0])
    next_point_on_original = 1

    # loop until we have considered all original points
    while next_point_on_original < len(line.coords):
        # the current point that we are checking from is the latest point appended to new list
        current_point = Point(new_points[-1])

        # get distance from current point to the next original point
        dist_to_next = current_point.distance(
            Point(line.coords[next_point_on_original])
        )

        if dist_to_next > max_interval:
            # if distance is greater than the max interval, interpolate a point max interval distance away from current point
            next_point = line.interpolate(line.project(current_point) + max_interval)
        else:
            # if distance is not greater than the max interval, add next original point to list
            next_point = Point(line.coords[next_point_on_original])

            # increment to consider the following original point along
            next_point_on_original = next_point_on_original + 1

        new_points.append(next_point)

    new_polygon = Polygon(new_points)
    return new_polygon


"""
Ring Functions
-
Headlands are processed as closed coordinate rings, implemented with LineStrings (because necessary functions are not available for the LinearRing object).
A headland ring should always be continguous around, with the last coordinate equalling the first coordinate.
Maintaining the structure of this LineString as a contiguous ring, and performing operations based on this structure, is done by the functions below.
"""


# shuffles indices around a ring
def reorder_ring(ring, new_begin):
    if new_begin == len(ring.coords) - 1:
        # last coord in ring is the same as first coord, so no re-ordering necesssary
        return ring

    # slice off currently repeated last coord
    # reconstruct ring with new begin point at index 0
    # close the ring by appending a repetition of the first coord to the end
    return LineString(
        ring.coords[new_begin:-1] + ring.coords[:new_begin] + [ring.coords[new_begin]]
    )


# get length of a segment along a ring between 2 indices
def dist_between_ring_indicies(ring, idx1, idx2):
    if idx1 == idx2:
        return 0
    elif idx1 > idx2:
        start = idx2
        end = idx1
    else:
        start = idx1
        end = idx2

    segment = LineString(ring.coords[start : end + 1])
    dist = segment.length

    # if segment length is more than half of total length, take segment other way around ring instead
    if dist > ring.length / 2:
        # because last coord in ring is same as first coord, it can be sliced off
        segment = LineString(ring.coords[end:-1] + ring.coords[0 : start + 1])
        dist = segment.length

    return dist


# wrap an index around a ring if it's less than 0 or greater than ring length
def wrap_ring_idx(i, length):
    # because last coord in ring is same as first coord, do not consider it as unique index point while wrapping
    if i > length - 2:
        i = i - (length - 1)
    elif i < 0:
        i = (length - 1) + i
    return i


# if a ring has been split at any point other than its begin/end join, it will result in 1 extra segment than expected
# of the resulting segments that would be expected from a split function, one will be further split in 2 at the ring's original begin/end point
# this function rejoins the extra split segment back into a single segment, given the ring's original begin/end point, and returns as a list
def rejoin_split_ring(segments, ring_join_coord):
    # if split "segments" only contain a single segment already, just return it in a list
    if isinstance(segments, LineString):
        return [segments]

    rejoined_segments = list()
    join = list()
    for segment in segments.geoms:
        if (
            segment.coords[0] == ring_join_coord
            or segment.coords[-1] == ring_join_coord
        ):
            join.append(segment)
        else:
            rejoined_segments.append(segment)

    if len(join) != 2:
        raise AlgorithmError("Ring join coord provided is not found in split segments.")

    joined = linemerge(join)
    rejoined_segments.append(joined)

    return rejoined_segments


# split ring and rejoin extra split segment, return as list of segments
def split_ring(ring, split_by):
    return rejoin_split_ring(split(ring, split_by), ring.coords[0])


# given the interpolated coords of a point on the ring, insert it into the ring so it can be accessed by index in future
def insert_interp_point(ring, point):
    point_dist = ring.project(point)

    # index of last true coord on ring is 1 from the end, since last coord is same as first
    last_idx = len(ring.coords) - 1

    already_inserted = False
    if point_dist > ring.project(Point(ring.coords[last_idx - 1])):
        # if point is past the end of the ring
        new_idx = last_idx
    else:
        # if point is not past the end of the ring, iterate until the the index of the next furthest point
        for i in range(last_idx):
            curr_dist = ring.project(Point(ring.coords[i]))
            if curr_dist == point_dist:
                already_inserted = True
                new_idx = i
                break
            if curr_dist > point_dist:
                new_idx = i
                break

    if already_inserted:
        new_ring = ring
    else:
        new_ring = LineString(ring.coords[0:new_idx] + [point] + ring.coords[new_idx:])

    # return index and new ring with point projected in at index
    return new_idx, new_ring


# given the indicies of an interval on a ring, interpolate the points a given distance to either side
# insert those interpolated points into the ring so they can be accessed by index in future
# if interpolating around a point instead of an interval, function can be called with idx1 == idx2 (i.e. an interval of length 0)
def get_insert_interp_points_at_dist(ring, idx1, idx2, dist1, dist2=None):
    if dist2 is None:
        dist2 = dist1

    if dist1 + dist2 > ring.length:
        raise AlgorithmError("Not enough length to project further.")

    if idx1 < idx2:
        start = idx1
        end = idx2
    else:
        start = idx2
        end = idx1

    # reorder ring to begin at interval start index
    shuffled_ring = reorder_ring(ring, start)

    # interpolate 1st point dist length before interval start
    p1 = shuffled_ring.interpolate(-dist1)
    insert_1 = 0

    # iterate backwards, not including ring end point which is equal to its start point
    # step through coords to find index to insert interpolated point at contiguously
    for i in range(len(shuffled_ring.coords) - 2, 0, -1):
        # as we are taking an interval going backwards, must use the ring distance function to calculate properly
        if dist_between_ring_indicies(shuffled_ring, 0, i) > dist1:
            insert_1 = i + 1 + start
            insert_1 = wrap_ring_idx(insert_1, len(shuffled_ring.coords))
            break

    # if interpolating around an interval of non-zero length, reorder ring to begin at interval end index
    if start != end:
        shuffled_ring = reorder_ring(ring, end)

    # interpolate 2nd point dist length after interval end
    p2 = shuffled_ring.interpolate(dist2)
    insert_2 = len(shuffled_ring.coords) - 1

    # iterate forwards, step through coords to find index to insert interpolated point at contiguously
    for i in range(len(shuffled_ring.coords) - 1):
        # as we are taking intervals going forwards now, just using projection distance is enough
        if shuffled_ring.project(Point(shuffled_ring.coords[i])) > dist2:
            insert_2 = i + end
            insert_2 = wrap_ring_idx(insert_2, len(shuffled_ring.coords))
            break

    # if 2nd point has higher index than 1st, swap interpolated points to maintain index1 < index2, but mark with boolean flag
    # crosses_join indicates that the shortest interval between the points crosses backwards over the ring join, not forwards along incremental indices
    if insert_2 < insert_1:
        insert_1, insert_2 = swap(insert_1, insert_2)
        p1, p2 = swap(p1, p2)
        crosses_join = True
    else:
        crosses_join = False

    # splice interpolated points into the coordinate list at appropriate indices
    new_ring = list(ring.coords)
    if insert_1 == 0:
        new_ring = (
            [p1.coords[0]]
            + new_ring[:insert_2]
            + [p2.coords[0]]
            + new_ring[insert_2:-1]
            + [p1.coords[0]]
        )
    else:
        new_ring = (
            new_ring[0:insert_1]
            + [p1.coords[0]]
            + new_ring[insert_1:insert_2]
            + [p2.coords[0]]
            + new_ring[insert_2:]
        )
    new_ring = LineString(new_ring)
    return new_ring, insert_1, insert_2 + 1, crosses_join


# given 2 indices to splice at, replace the ring segment between them with the new_path given
# maintains the contiguous ring structure where the last coord is equal to the first coord
def splice_ring(ring, new_path, idx1, idx2, crosses_join):
    # new_path begin and end coordinates must exactly match the original path segment they are replacing
    assert new_path.coords[0] == ring.coords[idx1]
    assert new_path.coords[-1] == ring.coords[idx2]

    if not crosses_join:
        # if interval goes forwards across incremental indices, replace new path indices maintaining original ring join point
        ring1 = ring.coords[0:idx1]
        ring2 = ring.coords[idx2 + 1 :]
        ring = LineString(ring1 + list(new_path.coords) + ring2)
    else:
        # if interval crosses backwards over the ring join, reverse new path in order to join it to ring end
        new_path_reversed = list(new_path.coords)
        new_path_reversed.reverse()

        # since the original join coord was in the interval being replaced, ring will be reordered
        new_coords = list(ring.coords[idx1 + 1 : idx2] + new_path_reversed)

        # manually append the first coord to the end to maintain ring structure
        new_coords.append(new_coords[0])

        ring = LineString(new_coords)

    # assert contiguous ring structure is maintained
    assert ring.coords[0] == ring.coords[-1]
    return ring


"""
Dubins Functions
"""


# given 2 indices at which to splice a new path to a headland, get dubins input bearings at those points
def get_splice_bearings(
    headland,
    splice_point1_idx,
    splice_point2_idx,
    crosses_join,
):
    length = len(headland.coords)

    # get angle between the splice point and the points on either side, in the direction point 1 -> point 2
    if not crosses_join:
        splice_bearing1 = get_angle(
            headland.coords[
                wrap_ring_idx(
                    splice_point1_idx - 1,
                    length,
                )
            ],
            headland.coords[splice_point1_idx],
        )
        splice_bearing2 = get_angle(
            headland.coords[splice_point2_idx],
            headland.coords[
                wrap_ring_idx(
                    splice_point2_idx + 1,
                    length,
                )
            ],
        )
    else:
        # if shortest segment point 1 -> point 2 crosses begin/end join of headland ring, take angles in opposite direction
        splice_bearing1 = get_angle(
            headland.coords[splice_point1_idx],
            headland.coords[
                wrap_ring_idx(
                    splice_point1_idx - 1,
                    length,
                )
            ],
        )
        splice_bearing2 = get_angle(
            headland.coords[
                wrap_ring_idx(
                    splice_point2_idx + 1,
                    length,
                )
            ],
            headland.coords[splice_point2_idx],
        )
    return splice_bearing1, splice_bearing2


def get_dubins_path(p1, angle1, p2, angle2, radius):
    path = dubins.shortest_path(
        (
            *p1,
            angle1,
        ),
        (
            *p2,
            angle2,
        ),
        radius,
    )
    path_coords, _ = path.sample_many(path_generation_precision_m)
    return list(zip(*path_coords))[0:2]


# generate a dubins path between points at 2 given indices on a headland
def get_dubins_between_2(
    radius,
    headland,
    splice_point1_idx,
    splice_point2_idx,
    crosses_join,
):
    splice_bearing1, splice_bearing2 = get_splice_bearings(
        headland,
        splice_point1_idx,
        splice_point2_idx,
        crosses_join,
    )

    new_path_coords = get_dubins_path(
        headland.coords[splice_point1_idx],
        splice_bearing1,
        headland.coords[splice_point2_idx],
        splice_bearing2,
        radius,
    )

    # path is returned in separate x and y coord lists, and doesn't include the destination point, so append and zip that
    new_path_x = list(new_path_coords[0]) + [headland.coords[splice_point2_idx][0]]
    new_path_y = list(new_path_coords[1]) + [headland.coords[splice_point2_idx][1]]
    new_path = LineString(zip(new_path_x, new_path_y))
    return new_path


# generate a dubins path between points at 2 given indices on a headland, passing through a 3rd point in between
# bearing through this 3rd centre point can be specified, or left None to calculate based on the other 2 calculated bearings
def get_dubins_between_3(
    radius,
    headland,
    splice_point1_idx,
    splice_point2_idx,
    crosses_join,
    centre_point,
    centre_bearing=None,
):
    splice_bearing1, splice_bearing2 = get_splice_bearings(
        headland,
        splice_point1_idx,
        splice_point2_idx,
        crosses_join,
    )

    # if centre bearing is not specified, find as average of other two bearings
    if centre_bearing is None:
        centre_bearing = (splice_bearing1 + splice_bearing2) / 2

    # whichever value is provided for centre bearing, it's usually calculated to specify the gradient angle through the centre point, but not the direction along that gradient
    # here we check and set the direction that fits a path going from splice point 1 -> splice point 2

    # first, get the alternative flipped candidate, 180° different to the current centre bearing
    flipped = normalise_angle(centre_bearing + pi)

    # get the direct angle from point 1 -> point 2
    angle_between = get_angle(
        headland.coords[splice_point1_idx],
        headland.coords[splice_point2_idx],
    )

    # find the candidate, between original and flipped, that is closest to the angle from splice point 1 -> splice point 2
    original_diff = abs(centre_bearing - angle_between)
    flipped_diff = abs(flipped - angle_between)

    # because this is a magnitude comparison that does not consider direction, treat oblique angles as equivalent to their conjugate
    if min(flipped_diff, 2 * pi - flipped_diff) < min(
        original_diff, 2 * pi - original_diff
    ):
        centre_bearing = flipped

    # because the centre point is directly on the path that may be exactly on the edge of a geofence or obstacle boundary, project it outwards by a tiny bit
    # this is to shift it just slightly away from the boundary, because paths generated right to the edge of what's legal are very likely to fail checks due to a slight deviation

    # get the normal to the centre bearing, in either direction
    normal = centre_bearing + pi / 2
    normal_flipped = centre_bearing - pi / 2

    projected_centre1 = (
        centre_point[0] + smoothing_projection_adjustment_dist_m * cos(normal),
        centre_point[1] + smoothing_projection_adjustment_dist_m * sin(normal),
    )

    projected_centre2 = (
        centre_point[0] + smoothing_projection_adjustment_dist_m * cos(normal_flipped),
        centre_point[1] + smoothing_projection_adjustment_dist_m * sin(normal_flipped),
    )

    # find the candidate that is further from the splice points
    projected_centre = (
        projected_centre1
        if Point(projected_centre1).distance(Point(centre_point))
        > Point(projected_centre2).distance(Point(centre_point))
        else projected_centre2
    )

    # generate 2 sets of dubins paths from splice point 1 -> centre and centre -> splice point 2
    new_path_coords1 = get_dubins_path(
        headland.coords[splice_point1_idx],
        splice_bearing1,
        projected_centre,
        centre_bearing,
        radius,
    )
    new_path_coords2 = get_dubins_path(
        projected_centre,
        centre_bearing,
        headland.coords[splice_point2_idx],
        splice_bearing2,
        radius,
    )

    # path are returned in separate x and y coord lists, and don't include the destination point, so append and zip that
    new_path_x = (
        list(new_path_coords1[0])
        + list(new_path_coords2[0])
        + [headland.coords[splice_point2_idx][0]]
    )
    new_path_y = (
        list(new_path_coords1[1])
        + list(new_path_coords2[1])
        + [headland.coords[splice_point2_idx][1]]
    )
    new_path = LineString(zip(new_path_x, new_path_y))
    return new_path


"""
GeoJSON Functions
"""


# assemble a list of individual geojson geometry objects into a dictionary of structure that can be directly converted to a geojson feature collection object
def make_feature_collection(geojsons):
    features = list()
    for geojson in geojsons:
        feature = dict()
        feature["type"] = "Feature"
        feature["geometry"] = geojson
        feature["properties"] = dict()

        features.append(feature)

    feature_collection = dict()
    feature_collection["type"] = "FeatureCollection"
    feature_collection["features"] = features

    return feature_collection


"""
AWS Functions
"""


def get_api_key():
    secret_name = "events!connection/headland-creation-mongo-api-key/19adb189-3862-4f26-9ad7-3080830c1556"
    region_name = "ap-southeast-2"

    session = boto3.session.Session()
    client = session.client(service_name="secretsmanager", region_name=region_name)

    try:
        get_secret_value_response = client.get_secret_value(SecretId=secret_name)
    except ClientError as e:
        # For a list of exceptions thrown, see
        # https://docs.aws.amazon.com/secretsmanager/latest/apireference/API_GetSecretValue.html
        raise e

    secret = get_secret_value_response["SecretString"]
    return json.loads(secret)["api_key_value"]


class HeadlandCreator:
    def __init__(
        self,
        settings,
        operating_area,
        obstacles,
        tree_rows,
        adjacent_tree_rows,
        executionArn,
        skip_final_simplification=False,
        skip_progress_update=False,
        print_output=True,
    ):
        self.executionArn = executionArn
        self.print_output = print_output

        # for testing only, we have the option to not simplify so that we can test the original unsimplified result
        # this is a bit strange since that means we're not testing the actual output, but this is because simplification introduces variance that the turn radius calculation method cannot handle
        # we have not identified a good way to account for the precision loss due to simplification while still correctly detection legitimately problematic turn points
        # so, we just verify that the unsimplified result is good, and trust that the simplified result is also good even if we can't measure it
        self.skip_final_simplification = skip_final_simplification

        # also when running locally, we have the option to disable progress updates which will hang if there is no internet connection
        self.skip_progress_update = skip_progress_update
        if not skip_progress_update:
            self.apiKey = get_api_key()

        # get settings
        self.num_headlands = settings["numHeadlands"]
        self.headland_path_buffering_m = settings["biggestHeadlandPathBuffering"]

        # note that for headlands, minimum turn radius is not the same as the AB-headland turn radius (which is a config value to make desirable AB line shapes, not a minimum)
        self.min_turn_radius_m = settings["minPathTurnRadius"]

        # buffering functions are configured with the number of segments with which to approximate a circle quadrant
        # calculate the lowest number required to achieve the path generation precision in the resulting arc, when buffering by the min turn radius
        # number of segments = (circumference / 4) / path generation precision = (pi * 2 * radius / 4) / path generation precision = (pi * radius / 2) / path generation precision
        self.buffer_quad_segments = ceil(
            self.min_turn_radius_m * (pi / 2) / path_generation_precision_m
        )

        # initialise gps to cartesian converter
        projection = settings["projection"]
        project_type = projection["type"].lower()
        if project_type == "topcon":
            zone = projection["zone"]
            hemisphere = projection["hemisphere"]
            self.proj_converter = Proj(
                f"+proj=utm +ellps=WGS84 +datum=WGS84 +units=m +no_defs +zone={zone} +{hemisphere.lower()}"
            )
        elif project_type == "john_deere":
            reference_point = projection["referencePoint"]
            self.proj_converter = Proj(
                "+proj=merc +ellps=WGS84 +lat_ts="
                + str(reference_point["coordinates"][1])
                + " +lon_0="
                + str(reference_point["coordinates"][0])
            )
        elif project_type == "trimble":
            reference_point = projection["referencePoint"]
            elevation = projection["elevation"]

            # WGS84 ellipsoid
            major_ellipsoid = 6378137
            minor_ellipsoid = 6356752.3142

            self.proj_converter = Proj(
                "+proj=tmerc +lat_ts="
                + str(reference_point["coordinates"][1])
                + " +lon_0="
                + str(reference_point["coordinates"][0])
                + " +a= "
                + str(major_ellipsoid + int(elevation))
                + " +b= "
                + str(minor_ellipsoid + int(elevation))
            )
        else:
            raise KeyError(
                'Projection data malforned in input payload. Projection type must be one of "TOPCON", "JOHN_DEERE", or "TRIMBLE.'
            )

        # get operating area
        # operating areas do in fact have interior borders, but when being sent as headlands input those holes should be represented as obstacles
        assert_is_exterior_only(
            operating_area["coordinates"],
            "Operating area representation should not have interior borders.",
        )
        self.operating_area = Polygon(
            self.get_utm_coords(operating_area["coordinates"][0])
        )

        # get obstacles -> payload passes not the raw obstacle boundaries, but the boundaries buffered out by the appropriate distance the robot can be away from them
        self.obstacles_plus_outer_coverage = list()
        for obstacle in obstacles:
            # now that we receive buffered obstacle rings and not raw obstacles, buffered geometries may well have internal rings
            # in many cases we can ignore them because any disconnected areas inside other buffered obstacle rings can't possibly be reached
            # HOWEVER, there is one edge case that an entire operating area can be surrounded by obstacles, and the entire headland can fit inside it
            holes = list()
            for i in range(1, len(obstacle["coordinates"])):
                holes.append(self.get_utm_coords(obstacle["coordinates"][i]))
            self.obstacles_plus_outer_coverage.append(
                Polygon(self.get_utm_coords(obstacle["coordinates"][0]), holes=holes)
            )

        # treat adjacent tree rows the same as obstacles
        for tree_row_multi in adjacent_tree_rows:
            tree_row_buffering = tree_row_multi["widths"]["biggest"][
                "desiredBufferDistance"
            ]
            for tree_single in tree_row_multi["geometry"]["coordinates"]:
                tree_row_line = LineString(self.get_utm_coords(tree_single))
                tree_row_poly = tree_row_line.buffer(tree_row_buffering)
                self.obstacles_plus_outer_coverage.append(tree_row_poly)

        # get tree rows -> payload passes in the raw linestring, but to process similarly to obstacles we buffer them out by the appropriate distance the robot can be away from them
        self.tree_rows_plus_outer_coverage = list()
        for tree_row_multi in tree_rows:
            tree_row_buffering = tree_row_multi["widths"]["biggest"][
                "desiredBufferDistance"
            ]
            for tree_single in tree_row_multi["geometry"]["coordinates"]:
                tree_row_line = LineString(self.get_utm_coords(tree_single))
                tree_row_poly = tree_row_line.buffer(tree_row_buffering)
                self.tree_rows_plus_outer_coverage.append(tree_row_poly)

    # roughly estimate a progress percentage and post it online, so some kind of progress bar can be updated when running in lambda
    def update_progress(self):
        if self.skip_progress_update:
            return

        self.progress = self.progress + 1

        try:
            # Make the POST request with basic authentication
            endpoint = "https://ap-southeast-2.aws.data.mongodb-api.com/app/data-ocmbd/endpoint/data/v1/action/updateOne"

            body = {
                "dataSource": "swarmfarm-robotics",
                "database": "swarmbot-data",
                "collection": "headland-creation",
                "filter": {"executionArn": self.executionArn},
                "update": {
                    "$set": {
                        "status": "pending",
                        "progress": (self.progress / self.total_progress) * 100,
                    }
                },
                "upsert": True,
            }
            response = requests.post(
                endpoint, json=body, headers={"apiKey": self.apiKey}
            )

            # Check if the request was successful
            if response.status_code == 200:
                print(
                    f"Updating progress {self.progress} out of {self.total_progress}, {((self.progress / self.total_progress) * 100):.3f}%. Number posted successfully."
                )
            else:
                print(f"Failed to post number. Status code: {response.status_code}")
        except requests.RequestException as e:
            print(f"An error occurred: {e}")

    # convert from lat/long to utm x/y
    def get_utm_coords(self, geopoints):
        coords = list()
        for point in geopoints:
            utm = self.proj_converter(point[0], point[1])
            coords.append(utm)
        return coords

    # convert from utm x/y to lat/long
    def get_geopoints(self, coords):
        geopoints = list()
        for coord in coords:
            latlong = self.proj_converter(coord[0], coord[1], inverse=True)
            geopoints.append(latlong)
        return geopoints

    # convert utm coords to lat/long geojson
    def to_latlong_geojson(self, coords, type):
        return json.loads(to_geojson(type(self.get_geopoints(coords))))

    # if buffering the operating area in for headlands forms a multipolygon, set the operating area going forwards as only the largest subpolygon and mark the rest as unreachable
    def handle_unreachable_areas(
        self, effective_operating_area_minus_coverage, error_message
    ):
        # sort sub-polygons by area
        geoms = list()
        for geom in effective_operating_area_minus_coverage.geoms:
            # append as a tuple zipping area value with geom
            geoms.append((geom.area, geom))

        # sort by first tuple element of area
        geoms = sorted(geoms, reverse=True)

        # compile all but the largest sub-polygons into list of unreachable areas as geojson polygons
        unreachable_geojsons = list()
        for i in range(1, len(geoms)):
            unreachable_geojsons.append(
                self.to_latlong_geojson(
                    geoms[i][1].exterior.coords,
                    Polygon,
                )
            )
        error = Error(
            ErrorType.MALFORMED_OPERATING_AREA,
            error_message,
            geometry=make_feature_collection(unreachable_geojsons),
        )

        # continue on with just the largest sub-polygon as the effective operating area
        effective_operating_area_minus_coverage = geoms[0][1]

        return effective_operating_area_minus_coverage, error

    # return if the path exceeds the max turn angle, at the sampling distance, past the tolerance value
    def is_traversable(self, ring, idx):
        # reorder ring to start at specified idx to make interpolating distance easier
        shuffled_ring = reorder_ring(ring, idx)

        # take 2 points on either side of specific idx, at the sampling distance
        p1 = shuffled_ring.interpolate(-smoothing_sampling_dist_m)
        p2 = shuffled_ring.interpolate(smoothing_sampling_dist_m)

        # get angle between specified idx and points on either side
        bearing_1 = get_angle(ring.coords[idx], p1.coords[0])
        bearing_2 = get_angle(ring.coords[idx], p2.coords[0])
        angle_between = abs(pi - abs(bearing_2 - bearing_1))

        # for the given sampling distance, there is a max angle through the sampled points that can be calculated from the min turn radius, such that if angle_between is higher than this max then the turn is too sharp
        # the relationship between the max angle and the sample distance is taken by imagining the section of path as an arc of a circle, such that sample distance = arc length = min turn radius * central angle (in rads)
        # thus, max turn angle = smoothing_sampling_dist_m/min_turn_radius_m rads
        return (
            angle_between
            <= smoothing_sampling_dist_m / self.min_turn_radius_m
            + radians(smoothing_turn_angle_tolerance_deg)
        )

    # return if 2 points are part of a single untraversable segment -> if current point is untraversable, AND within sampling distance of previous point
    def is_untraversable_segment(self, headland, prev_idx, current_idx):
        return (
            not self.is_traversable(headland, current_idx)
            and dist_between_ring_indicies(headland, prev_idx, current_idx)
            <= smoothing_sampling_dist_m
        )

    # return if the path exceeds the effective operating area (operating area + outer obstacles) boundaries, past the tolerance value
    def in_operating_area(self, path):
        return self.effective_operating_area_minus_coverage_plus_tol.contains(path)

    # return if the path exceeds any obstacle boundaries, past the tolerance value
    def avoids_obstacles(self, path):
        return not self.all_obstacles_plus_outer_coverage_minus_tol.intersects(
            path
        ) and not self.all_tree_rows_plus_outer_coverage_minus_tol.intersects(path)

    # as well as not exceeding boundaries, smoothed paths should be strictly shorter than the original path length
    # accept if shorter than the smoothing distance projected in each direction, plus distance start to end of segment being smoothed
    def splice_smoothed_path_if_valid(
        self,
        new_path,
        smooth_dist,
        headland,
        i,
        j,
        new_headland,
        splice_point1_idx,
        splice_point2_idx,
        crosses_join,
    ):
        spliced = splice_ring(
            new_headland,
            new_path,
            splice_point1_idx,
            splice_point2_idx,
            crosses_join,
        )
        accept = (
            spliced.is_simple
            and self.in_operating_area(new_path)
            and self.avoids_obstacles(new_path)
            and new_path.length
            < (smooth_dist + dist_between_ring_indicies(headland, i, j))
        )
        return spliced if accept is True else None

    # as well as being safe to travel, filter out wildly long paths that result when dubins is struggling to fit the minimum turn radius
    # limit projected path length to the minimum distance between the points * a configurable maximum length factor
    def splice_projected_path_if_valid(
        self,
        new_path,
        new_inner_headland,
        splice_point1_idx,
        splice_point2_idx,
        projected_headland_coord,
        crosses_join,
    ):
        spliced = splice_ring(
            new_inner_headland,
            new_path,
            splice_point1_idx,
            splice_point2_idx,
            crosses_join,
        )
        accept = (
            spliced.is_simple
            and self.in_operating_area(new_path)
            and self.avoids_obstacles(new_path)
            and (
                new_path.length
                < (
                    dist_2d(
                        new_inner_headland.coords[splice_point1_idx],
                        projected_headland_coord,
                    )
                    + dist_2d(
                        new_inner_headland.coords[splice_point2_idx],
                        projected_headland_coord,
                    )
                )
                * path_max_extension_factor
            )
        )
        return spliced if accept is True else None

    def smooth_headland_if_valid(
        self,
        headland,
        idx1,
        idx2,
        new_headland,
        splice_point1_idx,
        splice_point2_idx,
        crosses_join,
        splice_dists,
    ):
        new_path = get_dubins_between_2(
            self.dubins_radius,
            new_headland,
            splice_point1_idx,
            splice_point2_idx,
            crosses_join,
        )
        spliced_new_headland = self.splice_smoothed_path_if_valid(
            new_path,
            splice_dists[0] + splice_dists[1],
            headland,
            idx1,
            idx2,
            new_headland,
            splice_point1_idx,
            splice_point2_idx,
            crosses_join,
        )
        return spliced_new_headland

    def project_headland_if_valid(
        self,
        new_headland,
        splice_point1_idx,
        splice_point2_idx,
        centre_point,
        centre_bearing,
        crosses_join,
    ):
        new_path = get_dubins_between_3(
            self.dubins_radius,
            new_headland,
            splice_point1_idx,
            splice_point2_idx,
            crosses_join,
            centre_point,
            centre_bearing,
        )
        spliced_new_headland = self.splice_projected_path_if_valid(
            new_path,
            new_headland,
            splice_point1_idx,
            splice_point2_idx,
            centre_point,
            crosses_join,
        )
        return spliced_new_headland

    # given a multipolygon result from buffering in a shape with bottlenecks, generate connecting lines at closest points between them
    def get_multipolygon_connections(self, multipolygon):
        connections = list()
        connected = list()

        # calculate a lookup table of distances between subpolygons
        # create it both as a dictionary for easy reference as a list for iterating through later
        dists_dict = dict()
        all_dists_list = list()
        for i in range(len(multipolygon.geoms)):
            # create a reference list for subpolygon i
            dists_list = list()
            for j in range(len(multipolygon.geoms)):
                if i == j:
                    continue

                # key dictionary with a tuple of the 2 subpolygon indices
                if (j, i) in dists_dict:
                    dists_dict[(i, j)] = dists_dict[(j, i)]
                else:
                    dists_dict[(i, j)] = multipolygon.geoms[i].distance(
                        multipolygon.geoms[j]
                    )

                # append to list as a tuple zipping i-j distance with j value, to be sorted to find closest
                dists_list.append((dists_dict[(i, j)], j))

            # sort reference list for subpolygon i by the tuple's first element of i-j distance
            all_dists_list.append(sorted(dists_list))

        # for each subpolygon, find its closest other subpolygon by accessing its sorted distance list
        for i in range(len(multipolygon.geoms)):
            # closest subpolygon is element 0 of the sorted distance list, of which index number is the second element of the tuple
            closest_idx = all_dists_list[i][0][1]

            connection_key = sorted((i, closest_idx))
            if connection_key in connected:
                # if already connected, skip
                continue

            connections.append(
                make_connection_between(
                    multipolygon.geoms[i],
                    multipolygon.geoms[closest_idx],
                    self.buffer_quad_segments,
                )
            )

            # mark this pair as already connected
            connected.append(connection_key)

        # return as a single union object of all connections
        return unary_union(connections)

    # create headland by buffering in either a normal Polygon or a reconnected MultiPolygon, and handle the potential for the result to create a MultiPolygon
    # return headland as LineString
    def buffer_in_and_reconnect_multipolygon(self, polygon, buffer_dist, connections):
        # first try just buffering full distance
        buffered = polygon.buffer(-buffer_dist, quad_segs=self.buffer_quad_segments)
        if buffered.is_empty:
            raise GenerationError(
                ErrorType.INNER_HEADLAND_FAILURE,
                "No area remaining to make additional headland.",
            )

        # if original geometry wasn't already a reconnected multipolygon, and buffered geometry is a single Polygon, return as is
        # never do this with reconnected multipolygons because buffering removes connections that will be re-added, and whether the original buffer result is a polygon or not is irrelevant
        if connections is None and isinstance(buffered, Polygon):
            return to_line_string(buffered), None

        # if buffered result needs to be handled as a MultiPolyon, generate connections between the subpolygons to transform it into a single polygon
        if self.print_output:
            print("connecting multipolygon")
        step = connect_multipolyon_step_default_m

        while True:  # loop until break
            step_buffered_geom = polygon
            dist_remaining = buffer_dist
            reduce_step = False

            # buffer in incrementally and repeatedly generate connections each step
            # with a step size small enough, this should create a valid path connecting all subpolygons at their closest points within the operating area
            # a step size too big may result in not finding the correct closest connection points due to feature loss from buffering
            while not approx_equals(dist_remaining, 0):
                if dist_remaining > step:
                    current_buffer_dist = step
                else:
                    current_buffer_dist = dist_remaining
                dist_remaining = dist_remaining - current_buffer_dist

                step_buffered_geom = step_buffered_geom.buffer(
                    -current_buffer_dist, quad_segs=self.buffer_quad_segments
                )

                # this is a temporary processing geometry that allows us to test for and generate connections
                # it is NOT the final connected polygon as we will re-check at the end or any of those connections are no longer necessary and should be dropped
                geom_with_temp_connections = step_buffered_geom

                # if there are connections from a past iteration, merge the previous connections with the newly buffered geometry
                # connection geometries have to be maintained separtely and re-added after every buffering operation since they're always so small they'll be removed by buffering
                # previous connections will be processed by get_multipolygon_connections same as other subpolygons, allowing them to be reconnected
                if connections is not None:
                    geom_with_temp_connections = unary_union(
                        [geom_with_temp_connections, connections]
                    )

                # while the buffer result is a MultiPolygon, connect it iteratively into a single Polygon
                # because get_multipolygon_connections only connects the closest subpolygons, it may have to be called multiple times to connect them all
                # e.g. if you have A, B, C, and D, the first past might connect A<->B and C<->D, then the second connects B<->C
                while isinstance(geom_with_temp_connections, MultiPolygon):
                    new_connections = self.get_multipolygon_connections(
                        geom_with_temp_connections
                    )

                    # if we buffer out too far in one step, the closest point between 2 polygons may no longer be the place we can actually draw a connection
                    # in that case we'd end up with a connection outside the operating area, so test for that and restart processing with a smaller step size if required
                    # NOTE: only operating area has to be checked at this stage, not obstacles, as avoiding obstacles will be handled separately later
                    if not self.in_operating_area(new_connections):
                        step = step - connect_multipolyon_step_precision_m

                        # if step size cannot be reduced, subpolygons cannot be connected
                        # should theoretically never get here if configured correctly, try reducing connect_multipolyon_step_precision_m if hit
                        if step <= 0:
                            raise AlgorithmError("Cannot connect multipolygon.")

                        reduce_step = True
                        if self.print_output:
                            print(f"Reducing step to {step}")
                        break

                    # merge new connections with connections from previous iterations, this will merge chains of connections into a single connection object
                    if connections is None:
                        connections = new_connections
                    else:
                        connections = unary_union([connections, new_connections])

                    # update temporary processing geometry with new connections
                    geom_with_temp_connections = unary_union(
                        [geom_with_temp_connections, connections]
                    )

                if self.print_output:
                    print(f"buffered {buffer_dist - dist_remaining} out of {buffer_dist}")

                # break buffering loop if step has to be reduced
                if reduce_step:
                    break

            # if reached here not because the step has to be reduced, we have completed the full distance of buffering
            if not reduce_step:
                break

        # now, we test to see if any of our connections are attempting to connect too-small subpolygons that have actually been lost to buffering and no longer exist
        # however, sometimes we have areas too small to fit a fully buffered in inner headland, but too big to be obtain full coverage from solely the outer headland
        # this spot of missed coverage is unlikely to be coverable by the fillCoverage algorithm if it is on the other side of a bottleneck
        # so, we attempt to cover it by maintain connections to subpolygons that exist when buffered in half the total distance, which represent the amount of missed coverage by the outer headland
        # this connection will only be single a straight line to and from the missed coverage area, but it should be projected into somethign driveable by the makeTraversable algorithm
        half_buffered_geom = polygon.buffer(
            -buffer_dist / 2, quad_segs=self.buffer_quad_segments
        )

        # maintain MultiPolygon type for subsequent operations
        if not isinstance(step_buffered_geom, MultiPolygon):
            step_buffered_geom = MultiPolygon([step_buffered_geom])
        if not isinstance(half_buffered_geom, MultiPolygon):
            half_buffered_geom = MultiPolygon([half_buffered_geom])
        if not isinstance(connections, MultiPolygon):
            connections = MultiPolygon([connections])

        # there should only ever be a number of valid connections equal to the number of subpolygons - 1
        # if there are too many connections, filter by checking which still connect 2 existing subpolygons
        if len(connections.geoms) > len(half_buffered_geom.geoms) - 1:
            valid_connections = list()
            invalid_connections = list()
            for connection in connections.geoms:
                # if an intersections touches 2 fully buffered subpolygons, it is definitely required
                # if an intersection touches no fully buffered subpolygons, it is part of a chain that is entirely gone and definitely not required
                # if an intersection touches 1 fully buffered subpolygon, it should be preserved for full coverage if it touches 2 half buffered subpolygons
                # NOTE: does this logic miss any edge cases? to be kept an eye on
                intersection_count = 0
                half_intersection_count = 0
                for subpolygon in step_buffered_geom.geoms:
                    if connection.intersects(subpolygon):
                        intersection_count = intersection_count + 1
                for subpolygon in half_buffered_geom.geoms:
                    if connection.intersects(subpolygon):
                        half_intersection_count = half_intersection_count + 1
                if intersection_count >= 2 or (
                    intersection_count == 1 and half_intersection_count == 2
                ):
                    valid_connections.append(connection)
                else:
                    invalid_connections.append(connection)

            # NOTE: PRETTY SURE THIS CODE BELOW IS NEVER RELEVANT NOW THAT THE ALGORITHM HAS BEEN RESTRUCTURED, leave for now just in case it needs to be reinstated
            # # if there are 2 or more dropped connections, it may be because a single subpolygon between both of them is gone
            # # but if that subpolygon had multiple connections, those connections may be part of a longer connection chain between further subpolygons
            # # if that is the case, they should be maintained and, with the subpolygon between them gone, be connected to each other
            # for i in range(len(invalid_connections)):
            #     for j in range(i + 1, len(invalid_connections)):
            #         # to test if a pair of dropped connections should remain and be connected to each other, create the proposed connection
            #         connection_between = make_connection_between(
            #             invalid_connections[i],
            #             invalid_connections[j],
            #             self.buffer_quad_segments,
            #         )

            #         # NOTE: determining if connections should be connected to each other in a longer connection chain is not done by distance because it is not a reliable indicator
            #         # depending on paddock shape, if a subpolygon is being lost to buffering, its connections on either side may be very far apart
            #         # instead, a proposed connection chain is determined valid if it is not contained by the current geometry (i.e. connects subpolygons that aren't already connected)
            #         # and if it is within the operating area (i.e. PROBABLY it does not connect 2 subpolygons that are not meant to be connected)

            #         # theoretically, there can exist 2 possible edge cases:
            #         # 1. when proposed connection chain is within the operating area, but still does not correctly connect 2 subpolygons that the headland needs connected
            #         # 2. when a proposed connection chain does attempt to connect the correct 2 subpolygons, but isn't within the operating area due to the buffering step being too large
            #         # no. 2 is an issue because the logic to reduce buffering step if a connection exceeds boundaries, applied above, cannot be done so here as it would be circular logic
            #         # neither of these theoretical cases can currently be handled and would produced unexpected results, but may be investigated in future if an example is encountered

            #         if not half_buffered_geom.contains(
            #             connection_between
            #         ) and self.in_operating_area(connection_between):
            #             if self.print_output:
            #                 print(
            #                     "subpolygon gone, connecting connections on either side"
            #                 )

            #             # re-add individual connections as valid, if not already
            #             if invalid_connections[i] not in valid_connections:
            #                 valid_connections.append(invalid_connections[i])
            #             if invalid_connections[j] not in valid_connections:
            #                 valid_connections.append(invalid_connections[j])

            #             # also add new connection between them
            #             valid_connections.append(connection_between)

            if self.print_output:
                if len(valid_connections) < len(connections.geoms):
                    print(
                        f"subpolygon gone, dropped {len(connections.geoms) - len(valid_connections)} connections"
                    )

            # update connections to be valid connections only
            connections = unary_union(valid_connections)

        # return resulting headland as well as connection geometry to be separated maintained for any further buffering operations
        return (
            to_line_string(unary_union([step_buffered_geom, connections])),
            connections,
        )

    # diverts headland around obstacles
    def avoid_obstacles(self, headland):
        iterations = 0
        for i in range(len(self.obstacles_trees_plus_outer_coverage)):
            # is it important we generate paths to the full distance away from the obstacle but check for intersections with tolerance
            # otherwise we end up generating points with floating point values exactly on the edge of what's acceptable
            if headland.intersects(
                self.obstacles_trees_plus_outer_coverage_minus_tol[i][1]
            ):
                # only increment iteration count for obstacles that actually require processing
                iterations = iterations + 1
                if self.print_output:
                    print(f"obstacles {iterations} {time() - self.start:.2f}")

                # get a path all the way around obstacle
                obstacle_headland = LineString(
                    self.obstacles_trees_plus_outer_coverage[i][1].exterior.coords
                )

                # get segments where the original headland and the obstacle headland cross
                headland_segments = split_ring(headland, obstacle_headland)
                obstacle_segments = split_ring(obstacle_headland, headland)

                if len(obstacle_segments) == 2 and len(headland_segments) == 2:
                    # if the headland and obstacle path only intersect in 2 places, just decide which way to go around obstacle
                    candidate0 = obstacle_segments[0]
                    candidate1 = obstacle_segments[1]

                    # only checking of operating area required, as obstacle path is generated by nature to not cross any obstacles
                    candidate0_passable = self.in_operating_area(candidate0)
                    candidate1_passable = self.in_operating_area(candidate1)

                    # if either candidate is impassable take the other, else if both are valid pick the best one by a metric depending on type
                    if candidate0_passable and not candidate1_passable:
                        new_path = obstacle_segments[0]
                    elif candidate1_passable and not candidate0_passable:
                        new_path = obstacle_segments[1]
                    elif candidate0_passable and candidate1_passable:
                        if (
                            self.obstacles_trees_plus_outer_coverage_minus_tol[i][0]
                            == ObstacleTag.OBSTACLE
                        ):
                            # for normal obstacles we take the shorter path, and don't have to worry about coverage since the obstacle will receive its own plan ring later
                            prefer_candidate0 = candidate0.length < candidate1.length
                        elif (
                            self.obstacles_trees_plus_outer_coverage_minus_tol[i][0]
                            == ObstacleTag.TREE_ROW
                        ):
                            # but for tree rows we always stick to the outside, since ideally headlands should get full coverage of everything on the outside of the trees
                            # test this by checking the distance to the outer ring of the operating area
                            prefer_candidate0 = self.operating_area_edge.distance(
                                candidate0
                            ) < self.operating_area_edge.distance(candidate1)
                        else:
                            raise AlgorithmError("Unhandled obstacle tag ENUM.")

                        new_path = candidate0 if prefer_candidate0 else candidate1
                    else:
                        # should theoretically never get here
                        # any truly impassable obstacle should get considered as part of operating area, and handled before this point
                        raise AlgorithmError("Cannot avoid obstacle.")

                    # cut the headland where it crosses the obstacle path, should result in a single remaining segment
                    new_headland = rejoin_split_ring(
                        headland.difference(
                            self.obstacles_trees_plus_outer_coverage[i][1]
                        ),
                        headland.coords[0],
                    )
                    assert len(new_headland) == 1

                    # splice in the new path around the obstacle
                    headland = linemerge([new_headland[0], new_path])

                else:
                    # if there are multiple places where headland and obstacle path cross over, decide which to take for each segment
                    candidates = list()
                    for headland_segment in headland_segments:
                        # for each headland segment, find the obstacle path segment between the same 2 points and zip into tuple
                        obstacle_segment = get_segment_between(
                            obstacle_segments,
                            Point(headland_segment.coords[0]),
                            Point(headland_segment.coords[-1]),
                        )

                        # this method for navigating around an obstacle assumes that each segment from one headland corresponds to a segment from the other with the same start/end points
                        # however, this is not always the case if one of them loops back in on itself and re-intersects a segment of the other -> we end up with a segment from one corresponding to multiple from the other
                        # we cannot currently handle things if this assumption is broken, hopefully this is enough of an edge case it isn't hit often
                        if obstacle_segment is None:
                            raise GenerationError(
                                ErrorType.INNER_HEADLAND_FAILURE,
                                "Cannot find path around obstacle. This may happen with oddly-shaped obstacles or ones nears bottlenecks.",
                            )

                        candidates.append((headland_segment, obstacle_segment))

                    new_paths = list()
                    for candidate in candidates:
                        # check operating area for both, but obstacle path is generated by nature to not cross any obstacles
                        headland_candidate_valid = self.in_operating_area(
                            candidate[0]
                        ) and not self.obstacles_trees_plus_outer_coverage_minus_tol[i][
                            1
                        ].intersects(
                            candidate[0]
                        )
                        obstacle_candidate_valid = self.in_operating_area(candidate[1])

                        # if headland candidate is impassable take the obstacle path candidate, else otherwise always take headland candidate
                        # by this logic, if there is excess headland on the other side of an obstacle, it should always be taken to avoid cutting part of the headland
                        # though in some cases the excess headland on the other side may actually not be required to get full coverage, this logic will include it anyway
                        # can potentially be improved in future to generate more efficient paths
                        if headland_candidate_valid:
                            new_paths.append(candidate[0])
                        elif obstacle_candidate_valid and not headland_candidate_valid:
                            new_paths.append(candidate[1])
                        else:
                            # should theoretically never get here, if we have avoided hitting the exception above
                            # any truly impassable obstacle should get considered as part of operating area, and handled before this point
                            raise AlgorithmError("Cannot avoid obstacle.")

                    # new headland will be the merged result of all processed path segments
                    headland = linemerge(new_paths)

        # assert contiguous ring structure of headland is maintained
        assert headland.coords[0] == headland.coords[-1]
        return headland

    # tests the turn angle required to traverse every single coordinate of a headland, and smooths segments that fail
    # NOTE: this algorithm has a known limitation -> for the sake of performance, each unique point is only ever tested once for traversability, and any previously tested point is skipped in future checks
    # however, a point's traversability depends on the segments around it, which can be altered if the point itself isn't altered, so a previously confirmed traversable point may not remain traversable
    # we can handle this externally by repeatedly calling the entire function until no potentially overriding alterations are made, so return both the result and a 'done' flag to mark this
    def make_traversable(self, headland, connections=None):
        # mark if the headland returned is fully traversable, or if changes were made during this call that still need to be re-tested
        fully_traversable = True

        if do_smooth_by_buffering:
            # attempt the simple solution first of treating headland as a polygon, and smoothing by buffering in each direction and back
            # because this method only smooths angles in the shorter direction, it will fail in operating area and obstacles shapes where this exceeds boundaries
            # it will also fail if the headland is not a normal valid polygon of sufficient size that can be buffered
            # smoothing by this method is always guaranteed to be fully driveable, so keep the 'fully_traversable' flag as True if we succeed
            smoothed_concave = False
            smoothed_convex = False
            headland_polygon = Polygon(headland)

            # buffer inwards, then outwards, to smooth concave angles
            # don't try if headland is a reconnected multipolygons, as it should definitely fail and could possbly have unexpected behaviour
            if connections:
                headland_polygon_in = headland_polygon.buffer(
                    -self.buffer_smooth_distance
                )
                if isinstance(headland_polygon_in, Polygon):
                    # if buffering inwards does not form a multipolygon, continue with buffering outwards
                    headland_polygon_in_out = headland_polygon_in.buffer(
                        self.buffer_smooth_distance
                    )

                    if self.in_operating_area(
                        headland_polygon_in_out
                    ) and self.avoids_obstacles(headland_polygon_in_out):
                        # if resulting polygon does not exceed boundaries, accept
                        smoothed_concave = True
                        headland_polygon = headland_polygon_in_out
                        if self.print_output:
                            print("smoothed concave with buffering")

            # buffer outwards, then inwards, to smooth convex angles
            headland_polygon_out_in = headland_polygon.buffer(
                self.buffer_smooth_distance
            ).buffer(-self.buffer_smooth_distance)
            if self.in_operating_area(
                headland_polygon_out_in
            ) and self.avoids_obstacles(headland_polygon_out_in):
                # if resulting polygon does not exceed boundaries, accept
                smoothed_convex = True
                headland_polygon = headland_polygon_out_in
                if self.print_output:
                    print("smoothed convex with buffering")

            if smoothed_concave or smoothed_convex:
                # if either simple smoothing attempt was successful, update headland
                headland = to_line_string(headland_polygon)

                if smoothed_concave and smoothed_convex:
                    # if both simple smoothing attempts were successful, return now
                    return headland, fully_traversable

            # if simple smoothing was not succifient, continue with elaborate smoothing algorithm

        # when the min turn radius is equal to or larger than the coverage distance, it is more likely to generate paths that cut corners, therefore always use a step of 1 to maximum precision of options to try
        if self.min_turn_radius_m >= self.headland_path_buffering_m:
            dist_step = 1
        else:
            dist_step = smoothing_splice_dist_step_m

        iterations = 0
        tested = list()
        while True:  # loop until break
            iterations = iterations + 1
            if self.print_output:
                print(f"traversable {iterations} {time() - self.start:.2f}")

            # if the first coord is untraversable, find earliest coord that is traversable and reorder headland to begin there
            # this is because logic to find edges of untraversable segments doesn't handle segments crossing headland ring join point
            if not self.is_traversable(headland, 0):
                for idx in range(1, len(headland.coords)):
                    if not self.is_untraversable_segment(headland, idx - 1, idx):
                        break
                headland = reorder_ring(headland, idx)

            # number of points in headland at the beginning of the iteration, before any splicing
            current_len = len(headland.coords)

            # loop through each point on headland
            for idx1 in range(current_len):
                # skip points that have already been tested
                if headland.coords[idx1] in tested:
                    continue

                # mark current point tested
                tested.append(headland.coords[idx1])

                # test traversability
                if self.is_traversable(headland, idx1):
                    continue

                # if untraversable, if idx1 is not the last point, find furthest subsequent untraversable point
                # idx1 may be part of untraversable segment that can be smoothed at once, or if not, processing will continue with idx1 == jidx2
                if idx1 != len(headland.coords) - 1:
                    for idx2 in range(idx1 + 1, len(headland.coords)):
                        # returns if 2 points are part of a single untraversable segment -> if current point is untraversable, AND within sampling distance of previous point
                        if self.is_untraversable_segment(headland, idx2 - 1, idx2):
                            # if considered within same segment, mark point as also tested
                            tested.append(headland.coords[idx2])
                        else:
                            break

                    # since we have broken at the next traversable point, decrement to get the last untraversable point
                    idx2 = idx2 - 1
                else:
                    idx2 = idx1

                # for use in smoothing by projection, determine centre point of the untraversable segment, and the tangent angle to it based on the bearings on either side
                centre = round((idx1 + idx2) / 2)
                around_centre1, around_centre2 = get_splice_bearings(
                    headland,
                    centre,
                    centre,
                    False,
                )
                tangent = (around_centre1 + around_centre2) / 2

                # NOTE: this algorithm smooths the untraversable segment from idx1 -> idx2 by splicing a distance away either side and generating a replacement dubins path
                # depending on the dubins min turn radius, shape and severity of any given turn, and location of nearby boundaries, different distance pairs will be required to achieve a valid new path
                # the following loop tries different pairs of splice distance value until it find ones that can generate a valid path, iterating in a specific priority order, explained: file://./../docs/priority_iterators.md

                next_dist_start = min_smoothing_splice_dist_m
                smoothed = False
                for section in range(smoothing_splice_dist_interval_sections):
                    for higher_dist in range(
                        next_dist_start, max_smoothing_splice_dist_m, dist_step
                    ):
                        lower_dist_start = floor(
                            higher_dist
                            * (smoothing_splice_dist_interval_sections - section)
                            / smoothing_splice_dist_interval_sections
                        )

                        lower_dist_end = round(
                            higher_dist
                            * (smoothing_splice_dist_interval_sections - (section + 1))
                            / smoothing_splice_dist_interval_sections
                        )
                        if (
                            lower_dist_end
                            < smoothing_splice_dist_interval_sectioning_floor_m
                        ):
                            lower_dist_end = 0
                            next_dist_start = higher_dist + dist_step

                        interval_step = max(
                            round(higher_dist / smoothing_splice_dist_num_intervals), 1
                        )

                        for lower_dist in range(
                            lower_dist_start,
                            lower_dist_end,
                            -interval_step,
                        ):
                            if higher_dist + lower_dist > headland.length:
                                continue

                            dists = list()
                            dists.append((higher_dist, lower_dist))
                            if higher_dist != lower_dist:
                                dists.append((lower_dist, higher_dist))

                            for dist in dists:
                                # compute the splice points, and insert them into the line so they can be referenced by index in future
                                (
                                    new_headland,
                                    splice_point1_idx,
                                    splice_point2_idx,
                                    crosses_join,
                                ) = get_insert_interp_points_at_dist(
                                    headland, idx1, idx2, dist[0], dist[1]
                                )

                                # first, try smoothing by generating a path between the 2 spliced points only, which will be strictly shorter than the original
                                # this method smooths only in the direction towards the concave side of a turn, which may be invalid if moving the path in this direction exceeds a boundary
                                spliced_new_headland = self.smooth_headland_if_valid(
                                    headland,
                                    idx1,
                                    idx2,
                                    new_headland,
                                    splice_point1_idx,
                                    splice_point2_idx,
                                    crosses_join,
                                    dist,
                                )
                                if spliced_new_headland:  # returns None if not valid
                                    if self.print_output:
                                        print(
                                            f"made traversable with 2 points {dist[0]} {dist[1]}"
                                        )
                                    smoothed = True
                                    break

                                # if 2 point smoothing was not valid, try smoothing by projection, extending the path instead of shortening it by forcing it through the centre point of the original segment
                                # because it is difficult to predict what the best bearing through the centre point should be for any given path shape, try multiple different options for centre bearing
                                # leaving it None allows it to be calculated from averaging the 2 splice bearings on either side of the full path, also try the tangent and just matching the angles on each side
                                for centre_bearing in [
                                    None,
                                    tangent,
                                    around_centre1,
                                    around_centre2,
                                ]:
                                    spliced_new_headland = (
                                        self.project_headland_if_valid(
                                            new_headland,
                                            splice_point1_idx,
                                            splice_point2_idx,
                                            headland.coords[centre],
                                            centre_bearing,
                                            crosses_join,
                                        )
                                    )
                                    if spliced_new_headland:
                                        smoothed = True
                                        if self.print_output:
                                            print(
                                                f"made traversable with 3 points {dist[0]} {dist[1]}"
                                            )
                                        break
                                if smoothed:
                                    break
                            if smoothed:
                                break
                        if smoothed:
                            break
                    if smoothed:
                        break

                if not smoothed:
                    if self.print_output:
                        print("!! cannot make traversable")
                    self.has_unsmoothable_points = True

                if smoothed:
                    headland = spliced_new_headland.simplify(
                        processing_simplification_precision_m
                    )

                    # if we have spliced in an altered path, we will need retesting to guarantee driveability for certain
                    fully_traversable = False

                    # break loop after smoothing as headland has changed and must be reprocessed
                    break

            # if we have looped through all headland coords without breaking, all points have been tested
            if idx1 == current_len - 1:
                break

            if iterations > traversable_timeout_loops:
                raise AlgorithmError(
                    "Timed out attempting to make headland traversable."
                )

        # assert contiguous ring structure of headland is maintained
        assert headland.coords[0] == headland.coords[-1]
        return headland, fully_traversable

    # tests for gaps in coverage between headlands, and extends segments outwards to cover them
    def fill_coverage(self, inner_headland, outer_headland):
        # coverage of previous headland
        outer_coverage = outer_headland.buffer(
            self.headland_path_buffering_m,
            quad_segs=self.buffer_quad_segments,
        )

        iterations = 0
        skipped_gaps = list()
        skipped_points = list()
        while True:  # loop until break
            iterations = iterations + 1
            if self.print_output:
                print(f"coverage {iterations} {time() - self.start:.2f}")

            # get coverage of current headland to merge with previous headland
            inner_coverage = inner_headland.buffer(
                self.headland_path_buffering_m,
                self.buffer_quad_segments,
            )

            # also merge in the current headland itself as a polygon to avoid the centre of the paddock being a "gap"
            coverage = unary_union(
                [
                    outer_coverage,
                    inner_coverage,
                    Polygon(inner_headland.coords),
                ]
            )

            # buffer coverage outward by half the tolerance and allowable missed coverage distance, which should close any holes the width of the total distance
            coverage = coverage.buffer((gap_tolerance_m + allow_missed_coverage_m) / 2)

            projected = False
            gaps = list()
            for gap_ring in coverage.interiors:
                gap = Polygon(gap_ring.coords)

                if gap in skipped_gaps:
                    continue

                # if gap contains an obstacle, subtract from the it the allowable coverage gap that can be left around an obstacle
                # this allowable coverage gap is calculated as the area that can be later covered by doing a separate obstacle headland
                if self.all_obstacles_plus_outer_coverage.intersects(gap):
                    true_gap = gap.difference(self.all_obstacle_headland_coverages)
                    if true_gap.is_empty:
                        # if gap is entirely eliminated, remove from processing
                        skipped_gaps.append(gap)
                        continue
                    else:
                        gap = true_gap

                gaps.append(
                    (
                        gap.distance(inner_headland.centroid)
                        * gap.distance(inner_headland),
                        gap,
                    )
                )

            gaps = sorted(gaps, reverse=True)
            for _, gap in gaps:
                # to find the point on the gap that is furthest away from the headland, construct a sortable list of dists from each point
                # append distance as a tuple zipped with the point index
                # after sorting, furthest point index is the second tuple element of the last list element
                gap_dists = list()
                if isinstance(gap, Polygon):
                    gap = MultiPolygon([gap])
                for i in range(len(gap.geoms)):
                    for j in range(len(gap.geoms[i].exterior.coords)):
                        gap_dists.append(
                            (
                                Point(gap.geoms[i].exterior.coords[j]).distance(
                                    inner_headland
                                ),
                                i,
                                j,
                            )
                        )
                gap_dists = sorted(gap_dists)
                furthest_point_to_cover = Point(
                    gap.geoms[gap_dists[-1][1]].exterior.coords[gap_dists[-1][2]]
                )

                # given the furthest point on the gap as the target to project coverage to, get the closest point on the headland to it
                closest_point_on_headland = nearest_points(
                    inner_headland, furthest_point_to_cover
                )[0]

                # insert the interpolated closest point into the LineString so it can be accessed by index in future
                closest_point_idx, inner_headland = insert_interp_point(
                    inner_headland, closest_point_on_headland
                )

                # because exact gap areas can change with changes around it, also check if just furthest point is already marked uncoverable
                if furthest_point_to_cover in skipped_points:
                    continue  # skip

                # get the projection direction bearing between the target coverage point and the closest point on the headland
                projection_bearing_towards_headland = get_angle(
                    furthest_point_to_cover.coords[0],
                    closest_point_on_headland.coords[0],
                )

                # project from point on headland along projection bearing towards target coverage point
                projected_headland_coord_towards_headland = (
                    furthest_point_to_cover.coords[0][0]
                    + self.projection_dist_with_adjustments
                    * cos(projection_bearing_towards_headland),
                    furthest_point_to_cover.coords[0][1]
                    + self.projection_dist_with_adjustments
                    * sin(projection_bearing_towards_headland),
                )

                # alternatively, get the projection direction bearing between the target coverage point and the centre of the gap to be filled
                projection_bearing_towards_centre = get_angle(
                    furthest_point_to_cover.coords[0],
                    gap.centroid.coords[0],
                )

                # project from point on headland along alternative projection bearing towards target coverage point
                projected_headland_coord_towards_centre = (
                    furthest_point_to_cover.coords[0][0]
                    + self.projection_dist_with_adjustments
                    * cos(projection_bearing_towards_centre),
                    furthest_point_to_cover.coords[0][1]
                    + self.projection_dist_with_adjustments
                    * sin(projection_bearing_towards_centre),
                )

                # NOTE: this algorithm projects the headland through the new point to be covered by splicing a distance away either side of the closest existing point and generating a replacement dubins path
                # depending on the dubins min turn radius, shape and severity of any given turn, and location of nearby boundaries, different distance pairs will be required to achieve a valid new path
                # the following loop tries different pairs of splice distance value until it find ones that can generate a valid path, iterating in a specific priority order, explained: file://./../docs/priority_iterators.md

                for section in range(1, coverage_splice_distance_sections + 1):
                    start = floor(
                        min_coverage_splice_dist_m
                        + (section - 1)
                        * (max_coverage_splice_dist_m - min_coverage_splice_dist_m)
                        / coverage_splice_distance_sections
                    )
                    end = ceil(
                        min_coverage_splice_dist_m
                        + section
                        * (max_coverage_splice_dist_m - min_coverage_splice_dist_m)
                        / coverage_splice_distance_sections
                    )

                    # iterate with the end range value inclusive
                    for interval in range(
                        floor((end - start) / coverage_splice_dist_step_m) + 1
                    ):
                        # save the dist2 list and splice points for the section so that they don't have to be recalculated
                        dist2s_saved = dict()
                        splice_saved = dict()
                        for dist1 in range(start, end + 1, coverage_splice_dist_step_m):
                            dist2s = list()

                            dist2 = dist1 - coverage_splice_dist_step_m * interval
                            if dist2 >= start and dist1 + dist2 < inner_headland.length:
                                dist2s.append(dist2)

                            if interval > 0:
                                dist2 = dist1 + coverage_splice_dist_step_m * interval
                                if (
                                    dist2 <= end
                                    and dist1 + dist2 < inner_headland.length
                                ):
                                    dist2s.append(
                                        dist1 + coverage_splice_dist_step_m * interval
                                    )

                            dist2s_saved[dist1] = dist2s

                            for dist2 in dist2s:
                                try:
                                    (
                                        new_inner_headland,
                                        splice_point1_idx,
                                        splice_point2_idx,
                                        crosses_join,
                                    ) = get_insert_interp_points_at_dist(
                                        inner_headland,
                                        closest_point_idx,
                                        closest_point_idx,
                                        dist1,
                                        dist2,
                                    )
                                except AlgorithmError as e:
                                    if self.print_output:
                                        print(e)
                                    break

                                splice_saved[(dist1, dist2)] = (
                                    new_inner_headland,
                                    splice_point1_idx,
                                    splice_point2_idx,
                                    crosses_join,
                                )

                                # calculate the bearings at the target points as the normal of the projection bearings
                                normal_towards_headland = normalise_angle(
                                    projection_bearing_towards_headland + pi / 2
                                )

                                # projection normal may not be the optimal centre bearing depending on shape of the gap, so try 2 options for centre bearing
                                # leaving it None allows it to be calculated from averaging the 2 splice bearings on either side of the full path
                                for projected_headland_coord, centre_bearing in [
                                    (
                                        projected_headland_coord_towards_headland,
                                        normal_towards_headland,
                                    ),
                                    (
                                        projected_headland_coord_towards_headland,
                                        None,
                                    ),
                                ]:
                                    spliced_new_headland = (
                                        self.project_headland_if_valid(
                                            new_inner_headland,
                                            splice_point1_idx,
                                            splice_point2_idx,
                                            projected_headland_coord,
                                            centre_bearing,
                                            crosses_join,
                                        )
                                    )
                                    if spliced_new_headland:
                                        projected = True
                                        print(
                                            f"filled coverage towards headland {dist1} {dist2}"
                                        )
                                        break
                                if projected:
                                    break
                            if projected:
                                break
                        if projected:
                            break

                        # if projection towards headland has failed for an entire section, retry with projecting towards gap centre instead, which can sometimes succeed where projection towards headland fails
                        # however, projecting towards gap centre is also likely to succeed where projecting towards headland would also eventually succeed at a higher distance, but with a worse/oddly-shaped path
                        # this is why we only try this at the end of a failed section instead of in the same loop as above
                        for dist1 in range(start, end, coverage_splice_dist_step_m):
                            for dist2 in dist2s_saved[dist1]:
                                (
                                    new_inner_headland,
                                    splice_point1_idx,
                                    splice_point2_idx,
                                    crosses_join,
                                ) = splice_saved[(dist1, dist2)]

                                # calculate the bearings at the target points as the normal of the projection bearings
                                normal_towards_centre = normalise_angle(
                                    projection_bearing_towards_centre + pi / 2
                                )

                                # projection normal may not be the optimal centre bearing depending on shape of the gap, so try 2 options for centre bearing
                                # leaving it None allows it to be calculated from averaging the 2 splice bearings on either side of the full path
                                for projected_headland_coord, centre_bearing in [
                                    (
                                        projected_headland_coord_towards_centre,
                                        normal_towards_centre,
                                    ),
                                    (
                                        projected_headland_coord_towards_centre,
                                        None,
                                    ),
                                ]:
                                    spliced_new_headland = (
                                        self.project_headland_if_valid(
                                            new_inner_headland,
                                            splice_point1_idx,
                                            splice_point2_idx,
                                            projected_headland_coord,
                                            centre_bearing,
                                            crosses_join,
                                        )
                                    )
                                    if spliced_new_headland:
                                        projected = True
                                        if self.print_output:
                                            print(
                                                f"filled coverage towards gap centre {dist1} {dist2}"
                                            )
                                        break
                                if projected:
                                    break
                            if projected:
                                break
                        if projected:
                            break
                    if projected:
                        break

                if not projected:
                    print("!! cannot fill coverage")
                    skipped_gaps.append(gap)
                    skipped_points.append(furthest_point_to_cover)
                    self.has_uncoverable_gaps = True

                if projected:
                    inner_headland = spliced_new_headland.simplify(
                        processing_simplification_precision_m
                    )

                    # break loop after splicing as headland has changed and must be reprocessed
                    break

            # if we have looped through all gaps without splicing, all fixable gaps have been processed
            if not projected:
                break

            if iterations > coverage_timeout_loops:
                raise AlgorithmError("Timed out attempting to fill coverage.")

        # assert contiguous ring structure of headland is maintained
        assert inner_headland.coords[0] == inner_headland.coords[-1]
        return inner_headland

    def generate_headlands(self):
        errors = list()
        self.has_uncoverable_gaps = False
        self.has_unsmoothable_points = False

        if self.print_output:
            self.start = time()

        # rough progress estimation, based on the first headland having 3 update points, each subsequent one having 5, and the final checks being 2 more
        self.progress = 0
        self.total_progress = 3 + (self.num_headlands - 1) * 5 + 2

        # add safety factors to minimum turn radius
        self.dubins_radius = self.min_turn_radius_m + dubins_radius_safety_dist_m
        self.buffer_smooth_distance = (
            self.min_turn_radius_m + buffer_smoothing_safety_dist_m
        )

        # subtract overlap distance from inner coverage distance
        self.inner_coverage_dist_with_overlap = (
            self.headland_path_buffering_m - headland_overlap_m
        )

        # for fill_coverage: distance to buffer current headland in order to calculate coverage, includes the gap tolerance distance
        self.inner_coverage_buffer_dist_plus_tol = self.headland_path_buffering_m

        # for fill_coverage: distance away from the gap to be covered to project the headland, includes all adjustment values
        self.projection_dist_with_adjustments = (
            self.inner_coverage_dist_with_overlap
            + allow_missed_coverage_m
            - gap_tolerance_m / 2
            - coverage_projection_extra_overlap_m
        )

        # get operating area edge as a LineString for use in some later operations
        self.operating_area_edge = LineString(self.operating_area.exterior.coords)

        # initialise effective operating area = operating area + untraversable obstacles
        effective_operating_area = self.operating_area

        # buffer effective operating area boundary in to exclude the boom distance that would have to be kept away form the true boundary
        self.effective_operating_area_minus_coverage = effective_operating_area.buffer(
            -self.headland_path_buffering_m, quad_segs=self.buffer_quad_segments
        )

        # validate effective operating area minus coverage
        if isinstance(self.effective_operating_area_minus_coverage, MultiPolygon):
            # if effective operating area forms a multipolygon, continue on with only the largest and mark the rest as unreachable
            (
                self.effective_operating_area_minus_coverage,
                error,
            ) = self.handle_unreachable_areas(
                self.effective_operating_area_minus_coverage,
                "Parts of the paddock are unreachable due to border shape.",
            )
            errors.append(error)

        # add tolerance factor for use in comparisons
        self.effective_operating_area_minus_coverage_plus_tol = (
            self.effective_operating_area_minus_coverage.buffer(
                geometric_precision_error_m, quad_segs=self.buffer_quad_segments
            )
        )

        # process obstacles

        # create merged single geometry of all obstacles for ease of later testing
        self.all_obstacles_plus_outer_coverage = unary_union(
            self.obstacles_plus_outer_coverage
        )

        # if there were multiple obstacles, re-split to combine adjacent obstacles (ones that cannot be traversed between) into a single geometry
        # we check first if there were multiple obstacles because if not this causes type issues
        if len(self.obstacles_plus_outer_coverage) > 1:
            if isinstance(self.all_obstacles_plus_outer_coverage, MultiPolygon):
                self.obstacles_plus_outer_coverage = list(
                    self.all_obstacles_plus_outer_coverage.geoms
                )
            else:
                self.obstacles_plus_outer_coverage = [
                    self.all_obstacles_plus_outer_coverage
                ]

        # for each combined obstacle, if there is insufficient distance to traverse between obstacle and operating area edge, remove from obstacle list and consider part of effective operating area
        centre_obstacles_plus_outer_coverage = list()
        for obstacle_plus_outer_coverage in self.obstacles_plus_outer_coverage:
            if not self.effective_operating_area_minus_coverage_plus_tol.contains(
                obstacle_plus_outer_coverage
            ):
                # if obstacle intersects operating area edge, subtract it from operating area to form effective operating area
                self.effective_operating_area_minus_coverage = (
                    self.effective_operating_area_minus_coverage.difference(
                        obstacle_plus_outer_coverage
                    )
                )
            else:
                # if not, add to centre obstacle list
                centre_obstacles_plus_outer_coverage.append(
                    obstacle_plus_outer_coverage
                )

        # if obstacles have been removed from list and added to effective operating area, recalculate resulting geometry
        if centre_obstacles_plus_outer_coverage != self.obstacles_plus_outer_coverage:
            self.obstacles_plus_outer_coverage = centre_obstacles_plus_outer_coverage
            self.all_obstacles_plus_outer_coverage = unary_union(
                self.obstacles_plus_outer_coverage
            )

            # re-validate effective operating area minus coverage after changes
            if isinstance(self.effective_operating_area_minus_coverage, MultiPolygon):
                # if effective operating area forms a multipolygon, continue on with only the largest and mark the rest as unreachable
                (
                    self.effective_operating_area_minus_coverage,
                    error,
                ) = self.handle_unreachable_areas(
                    self.effective_operating_area_minus_coverage,
                    "Parts of the paddock are unreachable due to impassable obstacle.",
                )
                errors.append(error)

            # re-generate effective operating area minus coverage plus tolerance factor after changes
            self.effective_operating_area_minus_coverage_plus_tol = (
                self.effective_operating_area_minus_coverage.buffer(
                    geometric_precision_error_m, quad_segs=self.buffer_quad_segments
                )
            )

            # re-calculate all obstacles single geometry after changes
            self.all_obstacles_plus_outer_coverage = unary_union(
                self.obstacles_plus_outer_coverage
            )

        # subtract tolerance factor from all obstacles single geometry
        self.all_obstacles_plus_outer_coverage_minus_tol = (
            self.all_obstacles_plus_outer_coverage.buffer(
                -geometric_precision_error_m, quad_segs=self.buffer_quad_segments
            )
        )

        # buffer out by coverage distance to compute the allowable coverage gap that can be left around an obstacle
        # this allowable coverage gap is calculated as the area that can be later covered by doing a separate obstacle headland
        self.all_obstacle_headland_coverages = (
            self.all_obstacles_plus_outer_coverage.buffer(
                self.headland_path_buffering_m, quad_segs=self.buffer_quad_segments
            )
        )

        # process tree rows

        # create merged single geometry of all tree rows for ease of later testing
        self.all_tree_rows_plus_outer_coverage = unary_union(
            self.tree_rows_plus_outer_coverage
        )

        # subtract tolerance factor from all tree rows single geometry
        self.all_tree_rows_plus_outer_coverage_minus_tol = (
            self.all_tree_rows_plus_outer_coverage.buffer(
                -geometric_precision_error_m, quad_segs=self.buffer_quad_segments
            )
        )

        # subtract tolerance factor from each obstacle
        self.obstacles_plus_outer_coverage_minus_tol = list()
        for obstacle_plus_outer_coverage in self.obstacles_plus_outer_coverage:
            self.obstacles_plus_outer_coverage_minus_tol.append(
                obstacle_plus_outer_coverage.buffer(
                    -geometric_precision_error_m, quad_segs=self.buffer_quad_segments
                )
            )

        for tree_row in self.tree_rows_plus_outer_coverage:
            self.obstacles_plus_outer_coverage_minus_tol.append(
                tree_row.buffer(
                    -geometric_precision_error_m, quad_segs=self.buffer_quad_segments
                )
            )

        # NOTE: plans do not drive rings around tree rows as they do with obstacles,therefore there is no corresponding geometry like 'all_obstacle_headland_coverages' for tree rows

        # for generating paths to divert around both obstacles and tree rows, collect tagged individual geometries with and without tolerance
        self.obstacles_trees_plus_outer_coverage = [
            (ObstacleTag.OBSTACLE, poly) for poly in self.obstacles_plus_outer_coverage
        ] + [
            (ObstacleTag.TREE_ROW, poly) for poly in self.tree_rows_plus_outer_coverage
        ]

        self.obstacles_trees_plus_outer_coverage_minus_tol = list()
        for tag, poly in self.obstacles_trees_plus_outer_coverage:
            self.obstacles_trees_plus_outer_coverage_minus_tol.append(
                (
                    tag,
                    poly.buffer(
                        -geometric_precision_error_m,
                        quad_segs=self.buffer_quad_segments,
                    ),
                )
            )

        # if there are any tree rows too close to the operating area edge, we raise an exception immediately because it will be impossible for a headland to fit all the way around
        # check this by do a difference and seeing if it's empty, other operations like 'contains' can fail on a borderline case even if the difference is empty
        impassable_intersections = self.all_tree_rows_plus_outer_coverage.difference(
            self.effective_operating_area_minus_coverage_plus_tol
        )
        if isinstance(impassable_intersections, MultiPolygon) or (
            isinstance(impassable_intersections, Polygon)
            and len(impassable_intersections.exterior.coords) > 0
        ):
            # create list of intersections between the tree rows and operating area as geojson polygons
            intersection_geojsons = list()
            if isinstance(impassable_intersections, Polygon):
                impassable_intersections = MultiPolygon([impassable_intersections])
            for i in range(len(impassable_intersections.geoms)):
                intersection_geojsons.append(
                    self.to_latlong_geojson(
                        impassable_intersections.geoms[i].exterior.coords, Polygon
                    )
                )

            raise GenerationError(
                error_type=ErrorType.MALFORMED_OPERATING_AREA,
                message="Tree Rows are too close to operating area edge, obstacles, or other tree rows to generate a headland around them. Consider adjusting the operating area or the tree row safety buffer.",
                geometry=make_feature_collection(intersection_geojsons),
            )

        if self.print_output:
            print(" * generating headland 0 * ")

        headlands = list()
        headlands_poly = list()

        # first headland is same as effective operating area minus coverage
        first_headland = to_line_string(self.effective_operating_area_minus_coverage)
        self.update_progress()

        # NOTE: the order of the following calls are important, due to the way the functions interact with each other and the assumptions they make about their own position in the flow
        # avoid_obstacles must be first, as make_traversable expects an input headland that already doesn't intersect obstacle boundaries

        # first, divert headland around obstacles
        first_headland = self.avoid_obstacles(first_headland)
        self.update_progress()

        # then, smooth so that the path is traversable within the turn radius limit
        for _ in range(max_make_traversable_iterations):
            first_headland, fully_traversable = self.make_traversable(first_headland)
            if fully_traversable:
                break
        self.update_progress()

        # save headland as LineString
        headlands.append(first_headland)

        # save Polygonised headland
        headlands_poly.append(Polygon(first_headland.coords))

        connections = None
        for inner in range(1, self.num_headlands):
            if self.print_output:
                print(f" * generating headland {inner} * ")

            # if anything fails in the generation of current headland, save error and return along with previously generated headlands
            try:
                # buffer in from outer headland polygon to generate next inner headland, and handle the potential for the result to create a MultiPolygon
                inner_headland, connections = self.buffer_in_and_reconnect_multipolygon(
                    headlands_poly[inner - 1],
                    self.inner_coverage_dist_with_overlap * 2,
                    connections,
                )
                self.update_progress()

                # NOTE: the order of the following calls are important, due to the way the functions interact with each other and the assumptions they make about their own position in the flow
                # avoid_obstacles must be first, as fill_coverage and make_traversable expect an input headland that already doesn't intersect obstacle boundaries
                # fill_coverage must be after make_traversable, because fill_coverage generates paths that are traversable, but make_traversable does not generate paths that have full coverage

                # first, divert headland around obstacles
                inner_headland = self.avoid_obstacles(inner_headland)
                self.update_progress()

                # from the buffered headland, test for and fill coverage gaps, then smooth any remaining untraversable points - > this sequence of calls is complicated and was arrived at through experimentation with test cases
                # the problem is that it's possible for these functions to get stuck correcting an unfixable segment back and forth -> fill_coverage introduces an creates turn then make_traversable creates a coverage gap
                # this can't be avoided for a certainty, but calling them alternately a few times reduces this chance, as well as making sure we set appropriate configs (see comments on configs above)
                # additionally, because fill_coverage generates corrected segments with dubins, it is much more likely to return a traversable result than make_traversable is to return a result without gaps
                # also, coverage gaps usually occur at corners, so projecting dubins corrections to fill them incidientally makes many corners traversable, meaning it is faster to call fill_coverage before make_traversable
                for i in range(max_make_traversable_iterations):
                    # if we repeatedly call this on a headland that already has full coverage, it wastes relatively little time
                    inner_headland = self.fill_coverage(
                        inner_headland, headlands[inner - 1]
                    )

                    # update once on the first fill_coverage call, which is likely to be the slowest
                    if i == 0:
                        self.update_progress()

                    # break the loop when make_traversable returns with fully_traversable = True, meaning we are guaranteed to leave this loop with traversable results
                    # we are NOT guaranteed to leave this loop without coverage gaps, but we have to call it at some point, and traversability is more important than full coverage
                    # at the very least we should call fill_coverage and make_traversable each twice
                    inner_headland, fully_traversable = self.make_traversable(
                        inner_headland
                    )

                    # update once on the first make_traversable call, which is likely to be the slowest
                    if i == 0:
                        self.update_progress()

                    if fully_traversable:
                        print(f"-- exiting coverage and traversable loop after {i + 1}")
                        break

                    # unlike other timeouts, this loop does not raise an exception if it hits its configured maximum number, because it is much more likely to have reached a reasonable result
                    # it seems likely it would just be stuck trying to correct back and forth between 2 minor imperfections, but overall the headland is still usable enough to return
                    if i == max_make_traversable_iterations - 1:
                        print(
                            "!! still trying to loop after hitting max_make_traversable_iterations"
                        )
                    self.has_unsmoothable_points = True

                self.update_progress()

                # save headland as LineString
                headlands.append(inner_headland)

                # save Polygonised headland
                headlands_poly.append(Polygon(inner_headland.coords))

            except (GenerationError, AlgorithmError) as e:
                error = e.error
                error.message = f"{error.message} Only {inner} headlands generated."
                errors.append(error)
                if self.print_output:
                    print(f"Error: {error.message}")
                break

        if self.has_uncoverable_gaps or self.has_unsmoothable_points:
            if self.print_output:
                print(
                    f"generation complete with potentially unsmoothable or uncoverage points, doing final checks"
                )

        # if there were any uncoverable gaps, re-test coverage again now, as a gap that failed once may have been covered by a later projection
        # if there were never any uncoverable gaps, we don't have to test again because fill_coverage will have guaranteed full coverage
        if self.has_uncoverable_gaps:
            gaps_geojsons = list()
            for headland_idx in range(1, len(headlands)):
                # get coverage of outer headland
                outer_coverage = headlands[headland_idx - 1].buffer(
                    self.headland_path_buffering_m, quad_segs=self.buffer_quad_segments
                )

                # get coverage of current headland to merge with previous headland
                inner_coverage = headlands[headland_idx].buffer(
                    self.headland_path_buffering_m,
                    self.buffer_quad_segments,
                )

                # also merge in the current headland itself as a polygon to avoid the centre of the paddock being a "gap"
                coverage = unary_union(
                    [
                        outer_coverage,
                        inner_coverage,
                        headlands_poly[headland_idx],
                    ]
                )

                coverage = coverage.buffer(
                    (gap_tolerance_m + allow_missed_coverage_m) / 2
                )

                for gap_ring in coverage.interiors:
                    gap = Polygon(gap_ring.coords)

                    # if gap contains an obstacle, subtract from the it the allowable coverage gap that can be left around an obstacle
                    # this allowable coverage gap is calculated as the area that can be later covered by doing a separate obstacle headland
                    if self.all_obstacles_plus_outer_coverage.intersects(gap):
                        true_gap = gap.difference(self.all_obstacle_headland_coverages)
                        if true_gap.is_empty:
                            continue
                        else:
                            gap = true_gap

                    if isinstance(gap, Polygon):
                        gap = MultiPolygon([gap])

                    for geom in gap.geoms:
                        gaps_geojsons.append(
                            self.to_latlong_geojson(geom.exterior.coords, Polygon)
                        )

            # if there are uncovered gaps, add an error to the list but only as a warning as they gaps aren't a safety concern -> we will still return the headlands anyway
            if len(gaps_geojsons) > 0:
                errors.append(
                    Error(
                        error_type=ErrorType.COVERAGE_WARNING,
                        message="WARNING: generated headlands contain coverage gaps that couldn't be filled in. Consider adjusting the operating area or any nearby obstacle boundaries.",
                        geometry=make_feature_collection(gaps_geojsons),
                    )
                )

        self.update_progress()

        # if there were any unsmoothable points, re-test traversability, as a point that failed once may have been removed by a later splicing operating
        # if there were never any unsmoothable points, we don't have to test again because make_traversable will have guaranteed full traversability
        if self.has_unsmoothable_points:
            unsafe_headlands_idxs = list()
            unsafe_headlands_geojsons = list()
            unsafe_points_geojsons = list()
            for headland_idx in range(len(headlands)):
                for coord_idx in range(len(headlands[headland_idx].coords)):
                    if not self.is_traversable(headlands[headland_idx], coord_idx):
                        if headland_idx not in unsafe_headlands_idxs:
                            unsafe_headlands_idxs.append(headland_idx)
                            unsafe_headlands_geojsons.append(
                                self.to_latlong_geojson(
                                    headlands[headland_idx].coords, LineString
                                )
                            )
                        unsafe_points_geojsons.append(
                            self.to_latlong_geojson(
                                [headlands[headland_idx].coords[coord_idx]], Point
                            )
                        )

            safe_headlands = list()
            safe_headlands_poly = list()
            for headland_idx in range(len(headlands)):
                if headland_idx not in unsafe_headlands_idxs:
                    safe_headlands.append(headlands[headland_idx])
                    safe_headlands_poly.append(headlands_poly[headland_idx])
            headlands = safe_headlands
            headlands_poly = safe_headlands_poly

            if len(unsafe_headlands_geojsons) > 0:
                # uncoverable points aren't a safety concern but unsmoothable points are-> while we still return headlands with uncoverable points, we move headlands with unsmoothable points into error geometries
                if self.tree_rows_plus_outer_coverage:
                    message = (
                        "Headland(s) could not be generated around tree rows with a safe traversable turn radius. Consider adjusting the operating area or any nearby obstacle boundaries.",
                    )
                else:
                    message = (
                        "Headland(s) could not be generated with a safe traversable turn radius. Consider adjusting the operating area or any nearby obstacle boundaries.",
                    )
                errors.append(
                    Error(
                        error_type=ErrorType.SMOOTHING_FAILURE,
                        message=message,
                        geometry=make_feature_collection(
                            unsafe_headlands_geojsons + unsafe_points_geojsons
                        ),
                    )
                )

        self.update_progress()

        # output headlands as polygons, and convert from utm back to gps
        headland_geojsons = list()
        for headland in headlands_poly:
            if not self.skip_final_simplification:
                headland = headland.simplify(final_simplification_precision_m)

            # correct to CCW winding order
            headland = orient(headland)

            # to avoid distortions on long straight lines caused by differing projection systems, interpolate to add points at a maximum interval apart
            # do this now on the final output because adding them early would only have them removed again during simplification
            headland = interpolate_polygon_at_interval(headland, max_point_interval_m)

            headland_geojsons.append(
                self.to_latlong_geojson(headland.exterior.coords, Polygon)
            )

        payload = dict()
        payload["headlands"] = make_feature_collection(headland_geojsons)

        # return errors as list of dictionaries labelled 'errors', instead of error-type objects
        error_return = make_error_list_return(errors)

        if self.print_output:
            print(f"| HEADLANDS GENERATED in {time() - self.start:.2f} seconds")

        return payload, error_return


if __name__ == "__main__":
    if len(argv) < 3:
        skip_final_simplification = False
        skip_progress_update = False
    else:
        skip_final_simplification = eval(argv[2])
        skip_progress_update = eval(argv[3])

    test_event = json.load(open(argv[1]))

    if "treeRows" in test_event:
        tree_rows_input = test_event["treeRows"]
    else:
        tree_rows_input = list()

    if "adjacentTreeRows" in test_event:
        adjacent_tree_rows_input = test_event["adjacentTreeRows"]
    else:
        adjacent_tree_rows_input = list()

    headland_creator = HeadlandCreator(
        settings=test_event["settings"],
        operating_area=test_event["operatingArea"],
        obstacles=test_event["obstacles"],
        tree_rows=tree_rows_input,
        adjacent_tree_rows=adjacent_tree_rows_input,
        executionArn="abc",
        skip_final_simplification=skip_final_simplification,
        skip_progress_update=skip_progress_update,
    )
    try:
        headlands, errors = headland_creator.generate_headlands()
        print(json.dumps(headlands))
        print(json.dumps(errors), file=stderr)
    except (GenerationError, AlgorithmError) as e:
        print(json.dumps(make_error_list_return(e)), file=stderr)