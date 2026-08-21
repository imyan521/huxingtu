# floorplan_cpp

The production C++/OpenCV floor-plan pipeline used by Android and the desktop
regression tool. `RunPipeline` is the only public entry point.

The pipeline requires four aligned inputs: the structural occupancy image, the
full visual point-cloud image, the semantic image, and optimized trajectory
points. It fits one trajectory-connected exterior, preserves the complete black
point cloud for presentation, detects verified internal partitions, and selects
the most consistent pruning branch.

Desktop build, if OpenCV development files are installed:

```bash
cmake -S floorplan_cpp -B /tmp/floorplan_build
cmake --build /tmp/floorplan_build
/tmp/floorplan_build/end_to_end_floor_plan \
  --input input.png \
  --visual visual.png \
  --semantic semantic.png \
  --trajectory trajectory.txt \
  --output output.png
```

For Android, this library should be compiled by NDK and linked with OpenCV
Android SDK, then exposed through JNI.
