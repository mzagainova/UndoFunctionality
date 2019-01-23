# Distributed_Collaborative_Task_Tree
1 - Object dropped - pub to /node_dropped topic - sets node.hold_status_.dropped = true

2 - In main update function - monitor hold_status_.dropped. If true, dialogue function called
3 - In dialogue function, pause arch & moveit, pub to dialogue topic and wait for response 
4 - If response true - set object to done, if false - undo object
5 - Unpause architecture & moveit 

