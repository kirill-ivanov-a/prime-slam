# Vocabulary
Here is the terminology used in the system
- **Key Object** — object that stands out in the image (for example, a point or a line).
- **Detector** — an object that identifies key objects in an image.
- **Descriptor** — some number or sequence corresponding to a specific key object to identify it.
- **Observation** — key object and its descriptor.
- **Matcher** — an object that establishes correspondence between two sets of descriptors.
- **Projector** — an object that projects key objects onto the image and back.
- **Pose** — a position and an orientation of a camera, either with respect to a fixed spatial reference frame, called the "world" frame, or with respect to another camera pose.
- **Absolute Pose** — a pose in the world reference frame.
- **Relative Pose** — a pose with respect to another pose.
- **Pose Estimator** — an object that estimates a pose, using a set of observations and objects in 3D against which to estimate the pose.
- **Frame** — an object that describes the state of the camera during a certain period of time. Includes a set of observations, sensor readings (the image captured by the camera), and an absolute pose.
- **Landmark** — an object describing the 3D position of a key object in the world coordinate system.
- **Map** — set of landmarks.
- **Mapper** — map-building object. 
- **Tracker** — an object that estimates the relative position of two frames and the absolute position of the frame relative to the map.
- **Trajectory** — sequence of absolute frame poses.
- **Frontend** — an object that refines the trajectory and map using new data from sensors.
- **Factor Graph** — an object describing the current state of the SLAM system.
- **Backend** — an object that optimizes the current state of the system using factor graph.
# Additional reading
- Cadena, Cesar, et al. "Past, present, and future of simultaneous localization and mapping: Toward the robust-perception age." IEEE Transactions on robotics 32.6 (2016): 1309-1332.
- Grisetti, Giorgio, et al. "A tutorial on graph-based SLAM." IEEE Intelligent Transportation Systems Magazine 2.4 (2010): 31-43.

