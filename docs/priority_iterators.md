# Priority Iterator Order, Logic, and Configuration

Both make_traversable and fill_coverage work by selecting a pair of slice distances, one on either side of a designated point, and generating a dubins path with the min turn radius between those points. This path is tested to see if it self-intersects, exceeds any obstacle or operating area boundaries, or is greater than the maximum length. If it passes all those tests, it is considered valid. If it does not, a different pair of slice distances is selected, the loop continues until a valid path is generated. As soon as a single valid path is found, the loop breaks. The definition of a valid path is based on those minimum criteria only, and does not guarantee that the path is actually an optimal one. There is no attempt made to find multiple possible valid paths and pick an optimal candidate between them, as many valid paths could be available and this would be very slow.

Two issues exist with this. One, the first valid path encountered could be suboptimal to the point of cutting corners, overly simplifying the shape of the headland, and/or losing too much coverage area. Two, even if a good path is eventually encountered, it may only be tried after a great many invalid paths are discarded, slowing down processing considerably.

The approach taken to address this problem is to try slice distance pairs in a order that both minimises the time to finding a valid path, and hopefully arrives at a more optimal valid path before attempting to try also valid but less optimal paths. The priority order, and therefore iteration logic, is different for make_traversable and fill_coverage, because the two different functionalities are optimised by different splice distance pairs

## make_traversable

We begin by defining the total range of distances to try, from min_smoothing_splice_dist_m to max_smoothing_splice_dist_m. The iterator moves up this range in steps of smoothing_splice_dist_step_m. For each step, the current distance is selected as the higher distance, with multiple intervals between the distance pair tried at different lower distances.

The total number of intervals tried is determined by smoothing_splice_dist_num_intervals, but iterated through in sections determined by smoothing_splice_dist_interval_sections. Because the distances tried only have integer precision, there is also defined a distance under which intervals ranges are no longer broken into sections because they would be too small, smoothing_splice_dist_interval_sectioning_floor_m.

Note that because distances are only taken to integer precision, depending on each distance value and whether or not it's an exact multiple of smoothing_splice_dist_num_intervals, the exact intervals being iterated through each section may not form even steps due to rounding. As the purpose of the algorithm is to try a number of distance pairs, which don't have to be precise to any specific pattern, this is mostly not an issue. Remain aware, however, that integer imprecision and rounding could potentially lead to odd iterator behaviour for some combinations of configurations, especially if values are very high, very low, or very close to each other.

For example, given:
min_smoothing_splice_dist_m = 1
max_smoothing_splice_dist_m = 20
smoothing_splice_dist_step_m = 3
smoothing_splice_dist_interval_sections = 2
smoothing_splice_dist_num_intervals = 4
smoothing_splice_dist_interval_sectioning_floor_m = 5

| Section    | Higher Dist (1 - 20, step of 3)             | Lower Dist Upper Limit | Lower Dist Lower Limit                                                     | Interval (step of 1/4 max dist) | Resulting Pair(s)  |
| ---------- | ------------------------------------------- | ---------------------- | -------------------------------------------------------------------------- | ------------------------------- | ------------------ |
| 1 out of 2 | 1                                           | 1                      | 0                                                                          | 0                               | (1, 1)             |
| 1 out of 2 | 4                                           | 4                      | 0 (because floor(4 / interval_sections) = 2 < interval_sectioning_floor_m) | 0                               | (4, 4)             |
| 1 out of 2 | 4                                           | 4                      | 0                                                                          | 1                               | (4, 3), (3, 4)     |
| 1 out of 2 | 4                                           | 4                      | 0                                                                          | 2                               | (4, 2), (2, 4)     |
| 1 out of 2 | 4                                           | 4                      | 0                                                                          | 3                               | (4, 1), (1, 4)     |
| 1 out of 2 | 7                                           | 7                      | 0 (because floor(7 / interval_sections) = 3 < interval_sectioning_floor_m) | 0                               | (7, 7)             |
| 1 out of 2 | 7                                           | 7                      | 0                                                                          | 2                               | (7, 5), (5, 7)     |
| 1 out of 2 | 7                                           | 7                      | 0                                                                          | 4                               | (7, 3), (3, 7)     |
| 1 out of 2 | 7                                           | 7                      | 0                                                                          | 6                               | (7, 1), (1, 7)     |
| 1 out of 2 | 10                                          | 10                     | 5                                                                          | 0                               | (10, 10)           |
| 1 out of 2 | 10                                          | 10                     | 5                                                                          | 2                               | (10, 8), (8, 10)   |
| 1 out of 2 | 10                                          | 10                     | 5                                                                          | 4                               | (10, 6), (6, 10)   |
| 1 out of 2 | 13                                          | 13                     | 6                                                                          | 0                               | (13, 13)           |
| 1 out of 2 | 13                                          | 13                     | 6                                                                          | 3                               | (13, 10), (10, 13) |
| 1 out of 2 | 13                                          | 13                     | 6                                                                          | 6                               | (13, 7), (7, 13)   |
| 1 out of 2 | 16                                          | 16                     | 8                                                                          | 0                               | (16, 16)           |
| 1 out of 2 | 16                                          | 16                     | 8                                                                          | 4                               | (16, 12), (12, 16) |
| 1 out of 2 | 19                                          | 19                     | 9                                                                          | 0                               | (19, 19)           |
| 1 out of 2 | 19                                          | 19                     | 9                                                                          | 5                               | (19, 14), (14, 19) |
| 2 out of 2 | lowest distance with untried intervals = 10 | 5                      | 0                                                                          | 5                               | (10, 5), (5, 10)   |
| 2 out of 2 | 10                                          | 5                      | 0                                                                          | 7                               | (10, 3), (3, 10)   |
| 2 out of 2 | 10                                          | 5                      | 0                                                                          | 9                               | (10, 1), (1, 10)   |
| 2 out of 2 | 13                                          | 6                      | 0                                                                          | 7                               | (13, 6), (6, 13)   |
| 2 out of 2 | 13                                          | 6                      | 0                                                                          | 10                              | (13, 3), (3, 13)   |
| 2 out of 2 | 16                                          | 8                      | 0                                                                          | 8                               | (16, 8), (8, 16)   |
| 2 out of 2 | 16                                          | 8                      | 0                                                                          | 12                              | (16, 4), (4, 16)   |
| 2 out of 2 | 19                                          | 9                      | 0                                                                          | 10                              | (19, 9), (9, 19)   |
| 2 out of 2 | 19                                          | 9                      | 0                                                                          | 15                              | (19, 4), (4, 19)   |

The reason why this order is desirable is that is distance pairs closer together produce more optimal paths than pair futher apart, but also distances that are generally shorter produce more optimal paths than distances which are longer. This iterator balances the two by iterating in both ascending distance but with pairs of a smaller intervals first.

## fill_coverage

We begin by defining the total range of distances to try, from min_coverage_splice_dist_m to max_coverage_splice_dist_m. The iterator moves up this range in steps of coverage_splice_dist_step_m. For each step, the current distance is taken as one of the distances, whereas the other is selected as the distance an increasingly far number of steps away. The number of coverage_splice_distance_sections determines how often the interval is incremented, with all the pairs at a certain interval being tried for a single section before moving onto the next section. This also means that distances will only be paired with distances within a single section, limiting the intervals that are tried.

Unlike make_traversable which seeks to replace a section of path with a similar, mostly equivalent path, fill_coverage seeks to replace a section of path with a significantly longer path that projects out further than the original. Because of the way it loops, fill_coverage also self-corrects if it has cut any corners (i.e. if one iteration cuts a corner and exposes a coverage gap, the next iteration will detect it and project again to cover it). Also, because fill_coverage only applied to inner headlands, it will have to navigate around obstacles but not around the operating area border. For these two reasons, it's easier to find splice distances for valid paths in fill_coverage than in make_traversable.

Thus, fill_coverage has a faster, less optimised iteration priority, and can have overall coaser configurations. Because in the majority of cases there are no obstacles to avoid, the most likely valid distance pair is with both distances the same, which is why it prioritises trying all pairs of interval 0 in a section first.

For example, given:
min_coverage_splice_dist_m = 3
max_coverage_splice_dist_m = 33
coverage_splice_dist_step_m = 5
coverage_splice_distance_sections = 3


| Section    | Interval | Dist 1 | Resulting Pair(s)  |
| ---------- | -------- | ------ | ------------------ |
| 1 out of 3 | 0        | 3      | (3, 3)             |
| 1 out of 3 | 0        | 8      | (8, 8)             |
| 1 out of 3 | 0        | 13     | (13, 13)           |
| 1 out of 3 | 5        | 3      | (3, 8)             |
| 1 out of 3 | 5        | 8      | (8, 3), (8, 13)    |
| 1 out of 3 | 5        | 13     | (13, 8)            |
| 1 out of 3 | 10       | 3      | (3, 13)            |
| 1 out of 3 | 10       | 8      | (13, 3)            |
| 2 out of 3 | 0        | 13     | (13, 13)           |
| 2 out of 3 | 0        | 18     | (18, 18)           |
| 2 out of 3 | 0        | 23     | (23, 23)           |
| 2 out of 3 | 5        | 13     | (13, 18)           |
| 2 out of 3 | 5        | 18     | (18, 13), (18, 23) |
| 2 out of 3 | 5        | 23     | (23, 18)           |
| 2 out of 3 | 10       | 13     | (13, 23)           |
| 2 out of 3 | 10       | 18     | (23, 13)           |
| 3 out of 3 | 0        | 23     | (23, 23)           |
| 3 out of 3 | 0        | 28     | (28, 28)           |
| 3 out of 3 | 0        | 33     | (33, 33)           |
| 3 out of 3 | 5        | 23     | (23, 28)           |
| 3 out of 3 | 5        | 28     | (28, 23), (28, 33) |
| 3 out of 3 | 5        | 33     | (33, 28)           |
| 3 out of 3 | 10       | 23     | (23, 33)           |
| 3 out of 3 | 10       | 28     | (33, 23)           |