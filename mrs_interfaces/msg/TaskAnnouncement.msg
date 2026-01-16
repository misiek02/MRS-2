int32 task_id
geometry_msgs/Point task_location    # formation center provided for MR tasks / desired position for SR tasks 
string task_type     # SR or MR
string task_formation       # for formation action request NOTE: # T - Triangle; S - Square; Lh - Line(horizontal); Lv - Line(vertical); A - alone
int32 num_required   # Number of robots required for the task
float32[] task_time  # [DU, ES, LF]
bool winner_selected
