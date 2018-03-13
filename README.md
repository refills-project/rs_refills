RoboSherlock components for Refills
------------------------------------

This package implements query-answering based perception for the refills scenario; It modifies the default query interface of RoboSherlock to handle perception tasks that last longer using the ``scan`` keyword;

Dependencies: 
 * robosherlock
 * rs_kbreasoning

Running:
 rosrun rs_refills processing_engine _ae:=refills

Querying:
 
 Start scanning a shelf_system for shelves:

``rosservice call /RoboSherlock_presentation/json_query "query: '{\"scan\":{\"type\":\"shelf\",\"command\":\"start\"}}'"``

 Stop scanning: 

``rosservice call /RoboSherlock_presentation/json_query "query: '{\"scan\":{\"type\":\"shelf\",\"command\":\"stop\"}}'"``
