import math

class ErrorState:
    def __init__(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.current_error = 0.0

    def update(self, error, dt):
        self.derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        self.integral += 0.5 * (error + self.previous_error) * dt
        self.previous_error = self.current_error
        self.current_error = error

    def __repr__(self):
        return (f"Error: {self.current_error}, "
                f"Integral: {self.integral}, "
                f"Derivative: {self.derivative}")


def PID_controller(error_state, dt, Kp, Ki, Kd):
    """
    PID controller implementation.

    Parameters:
    - error_state: An instance of ErrorState containing the current error, accumulated_error and .
    - dt: Time interval since the last update.
    - Kp: Proportional gain.
    - Ki: Integral gain.
    - Kd: Derivative gain.

    Returns:
    - control_signal: The control signal to be applied.
    """

    # PID formula
    control_signal = (Kp * error_state.current_error +
                      Ki * error_state.integral +
                      Kd * error_state.derivative)

    return control_signal

def output_controls(target, current, error_ref, error_intermediate, dt, gains_x, gains_y, gains_z): 
    """
    Function to compute the control signals for the quadrotor.
    Parameters:
    - target: The target state of the quadrotor. 
    - current: The current state of the quadrotor. Order of the 12 states are (x, y, z, U, V, W, p, q, r, phi, theta, psi).
    - error_ref: The reference error state.
    - error_intermediate: The intermediate error state.
    - dt: Time interval since the last update.
    - gains_x: The PID gains for the x-axis.
    - gains_y: The PID gains for the y-axis.
    - gains_z: The PID gains for the z-axis.
    Returns:
    - control_signal_x: Alpha gimbal for x direction control.
    - control_signal_y: Beta gimbal for y direction control.
    - control_signal_z: Thrust for height control.
    """
    
    error = target - current

    for i in range (3):
        error_ref[i].update(error[i], dt)

    desired_theta = PID_controller(error_ref[0], dt, gains_x[0], gains_x[1], gains_x[2]) 
    desired_phi = PID_controller(error_ref[1], dt, gains_y[0], gains_y[1], gains_y[2])

    #saturate the desired angles to a maximum of 10 degrees
    if desired_theta > math.radians(10):
        desired_theta = math.radians(10)
    elif desired_theta < -math.radians(10):
        desired_theta = -math.radians(10)
    
    if desired_phi > math.radians(10):
        desired_phi = math.radians(10)
    elif desired_phi < -math.radians(10):
        desired_phi = -math.radians(10)
        
        
    error_intermediate[0].update(desired_theta - current[11], dt)
    error_intermediate[1].update(desired_phi - current[10], dt)

    control_signal_x = PID_controller(error_intermediate[0], dt, gains_x[4], gains_x[5], gains_x[6]) # alpha
    control_signal_y = PID_controller(error_intermediate[1], dt, gains_y[4], gains_y[5], gains_y[6]) # beta
    control_signal_z = PID_controller(error_ref(3), dt, gains_z[0], gains_y[1], gains_y[2]) # thrust
    
    # staturate the control signals to a maximum of 5 degrees
    # and a minimum of -5 degrees
    if control_signal_x > math.radians(5):
        control_signal_x = 5
    elif control_signal_x < -math.radians(5):
        control_signal_x = -5
        
    if control_signal_y > math.radians(5):
        control_signal_y = 5
    elif control_signal_y < -math.radians(5):
        control_signal_y = -5
    
    # Convert to degrees, round to nearest integer, then convert back to radians
    control_signal_x = math.radians(round(math.degrees(control_signal_x)))
    control_signal_y = math.radians(round(math.degrees(control_signal_y)))
    
    if control_signal_z > 20:
        control_signal_z = 20
    elif control_signal_z < 0:
        control_signal_z = 0
        
    return control_signal_x, control_signal_y, control_signal_z

