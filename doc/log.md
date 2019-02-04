

---------------------------------------
02-02
Bundle adjustment has bug, I have to fix all points in order to get good result (and it indeed is better result compared to not using BA).
Is the bug due to following reasons:
* Is there a camera pose with 0 connection to the map point?  
    No.
* Is the graph all connected, or piece wise connected?    
    No. In my test cases, all are connected.
* Wrong information matrix?  
    No. I've tested [x, 0; 0, 1], with x=1e-5 and 1e5. However, both are wrong.
