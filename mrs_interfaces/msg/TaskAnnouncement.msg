int32 task_id
geometry_msgs/Point task_location    # formation center provided for MR tasks
string task_type     # SR or MR
float32[] offsets   # for MR tasks
int32 num_required   # Number of robots required for the task
float32[] task_time  # [DU, ES, LF]
bool winner_selected
