Pathfinder API
==============

.. autoclass:: pathfinder.Segment
    :members:
    :undoc-members:

.. autoclass:: pathfinder.Waypoint
    :members:
    :undoc-members:

.. autoclass:: pathfinder.TrajectoryInfo
    :members:
    :undoc-members:

.. autofunction:: pathfinder.generate
.. autofunction:: pathfinder.d2r
.. autofunction:: pathfinder.r2d
.. autofunction:: pathfinder.boundHalfDegrees


Followers
---------

.. autoclass:: pathfinder.followers.DistanceFollower
    :members:
    :undoc-members:

.. autoclass:: pathfinder.followers.EncoderFollower
    :members:
    :undoc-members:

Modifiers
---------

.. autoclass:: pathfinder.modifiers.SwerveModifier
    :members:
    :undoc-members:

.. autoclass:: pathfinder.modifiers.TankModifier
    :members:
    :undoc-members:

Serialization
-------------

For serializing/deserializing in python programs, it's probably easiest to use
Python's ``pickle`` module to directly serialize a trajectory::
    
    import pickle
    
    with open('fname', 'wb') as fp:
        pickle.dump(trajectory, fp)
        
    with open('fname', 'rb') as fp:
        trajectory = pickle.load(fp)

One advantage to this approach is that you could put multiple trajectories in a
data structure such as a dictionary, and serialize them all in a single file.
The pathfinder compatibility serialization routines only support a single
trajectory per file.

However, for compatibility with other pathfinder implementations, the following
functions are made available.

.. autofunction:: pathfinder.deserialize
.. autofunction:: pathfinder.deserialize_csv
.. autofunction:: pathfinder.serialize
.. autofunction:: pathfinder.serialize_csv
