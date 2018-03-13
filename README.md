RoboSherlock components for Refills
------------------------------------

This package implements query-answering based perception for the refills scenario; It modifies the default query interface of RoboSherlock to handle perception tasks that last longer using the ``scan`` keyword;

Dependencies: 
 * robosherlock
 * rs_kbreasoning

**Running(launch file comming soon):**
 rosrun rs_refills processing_engine _ae:=refills

**Querying:**
 
 Start scanning a shelf_system for shelves:

``rosservice call /RoboSherlock_presentation/json_query "query: '{\"scan\":{\"type\":\"shelf\",\"command\":\"start\"}}'"``

Returns empty string

 Stop scanning: 

``rosservice call /RoboSherlock_presentation/json_query "query: '{\"scan\":{\"type\":\"shelf\",\"command\":\"stop\"}}'"``

Returns a vector of object descritions. Each object description is a json string, e.g.:
```json
{
   "timestamp":1520355438.2960196,
   "id":"0",
   "class":{
      "confidence":0.0,
      "source":"ShelfDetector",
      "name":"0"
   },
   "pose":[
      {
         "source":"ShelfDetector",
         "pose":{
            "header":{
               "seq":0,
               "stamp":{
                  "sec":1520355438,
                  "nsec":296019620
               },
               "frame_id":"map"
            },
            "pose":{
               "position":{
                  "x":-0.9697583913803101,
                  "y":0.4238448441028595,
                  "z":1.4172399044036866
               },
               "orientation":{
                  "x":0.0,
                  "y":0.0,
                  "z":0.0,
                  "w":1.0
               }
            }
         }
      }
   ]
}
```
