int32[] agent_ids
int16 agent_count

# Buffer data for each agent (last 10 values for each field)
float32[] x_velocities    # Flattened array with up to 10 values per agent
float32[] y_velocities    # Flattened array with up to 10 values per agent
string[] class_ids         # Flattened array with up to 10 values per agent
float32[] x_positions     # Flattened array with up to 10 values per agent
float32[] y_positions     # Flattened array with up to 10 values per agent

# Statistical data for each agent
float32[] x_mean          # Mean x-velocity for each agent
float32[] y_mean          # Mean y-velocity for each agent
float32[] x_std_dev       # Standard deviation of x-velocity for each agent
float32[] y_std_dev       # Standard deviation of y-velocity for each agent
float32[] x_variance      # Variance of x-velocity for each agent
float32[] y_variance      # Variance of y-velocity for each agent
string[] majority_class_id # Most common class ID for each agent
