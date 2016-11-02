# initial_surface_view_evaluation

This system provides an action server for doing initial surface view evaluation to the surface-based object learning pipeline. The technique this package implements is intended to improve our ability to plan more meaningful views of surfaces by using prior information about the structure of the environment. In our previous approach, we selected views in such a way as to maximise *coverage* of a supporting surface. However, this does not take into account the spatial context of objects in relation to the surface during a given learning episode. In short, a good view that covers much of a surface may in fact just be a view of empty space, and we would prefer to select views that will be focused more on the areas of a surface where there are objects. This is what this package attempts to do.

The technique is as follows.

1. The robot moves to a waypoint and calls the action provided by this package.
2. The latest meta-room scan is retreived from disk, and filtered so that only points within the SOMA ROI at this waypoint are considered.
3. From here, we apply some radial noise and statistical outlier filters to smooth out the observation.
4. Next, we convert the observation to an octomap and return only voxels with an up-facing normal. This is a very simple approach to surface detection.
5. Given the surface, we calculate its centroid and point the PTU at it. From here, we execute a mini camera sweep at 30° intervals, taking three views. Each view is segmented, and placed into an octomap.
6. This then gives us an octomap of the objects that have been detected on the table, and we can use this as a base to begin taking more detailed views using view planning.
