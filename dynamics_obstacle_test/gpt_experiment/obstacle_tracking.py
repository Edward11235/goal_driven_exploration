
import numpy as np

def track_and_predict(prev_pos, curr_pos):
    velocity = np.array(curr_pos) - np.array(prev_pos)
    predicted_pos = np.array(curr_pos) + velocity
    return predicted_pos.astype(int).tolist()
