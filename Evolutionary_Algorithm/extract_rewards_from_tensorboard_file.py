from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
import os

def get_tb_data(tb_log_dir, fields):
    """
    This function will get the data from the tensorboard logs and return a list with the last value of each field.

    Args:
    tb_log_dir: base directory where the tensorboard logs are stored
    fields: list of fields to extract from the logs

    Returns:
    List of last values from the specified fields or [0, 0] if an error occurs.
    """
    try:
        data_array = []
        log_directory = tb_log_dir

        print('Loading data ...')
        event_acc = EventAccumulator(log_directory)
        event_acc.Reload()

        for field in fields:
            scalar = event_acc.Scalars(field)
            # get the last value and store it in the array if available
            if scalar:  # Check if there are any entries in scalar
                data_array.append(scalar[-1].value)
            else:
                data_array.append(0)  # Append 0 if no data is available for a field
        
        return data_array

    except Exception as e:
        print("An error occurred:", str(e))
        return [0, 0]  # Return [0, 0] if any exception is raised


def extract_velocity_and_energy_reward(offspring_path): # Intended to be called on robot.model_dir

    fields = ['Episode/rew_lin_vel_xy', 'Episode/rew_joint_pow_raw']

    path_to_runs_folder = os.path.join(offspring_path,'run', 'runs')

    if os.path.exists(path_to_runs_folder) and os.path.isdir(path_to_runs_folder):

        # List all entries in the directory
        directories = os.listdir(path_to_runs_folder)
        # Filter out only directories (ignore files)
        directories = [d for d in directories if os.path.isdir(os.path.join(path_to_runs_folder, d))]
        # Sort the directories alphabetically
        directories.sort()
        # Pop the last directory name (last alphabetically)
        specific_dir = directories.pop() if directories else None

        full_path_to_summaries_folder = os.path.join(path_to_runs_folder, specific_dir, 'summaries')

        print(full_path_to_summaries_folder)

        data = get_tb_data(full_path_to_summaries_folder, fields)

        print(data)
        # print("Hello World")
        return data  # in format [velocity_reward, power_reward]
    else:
        return [0,0]


offspring_path = './Experiment_3/test_vis/FrogBot_AntBot_top-z-3_FrogBot_bottom-y-8'
extract_velocity_and_energy_reward(offspring_path)