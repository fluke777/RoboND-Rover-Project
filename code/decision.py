import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status

        if Rover.mode == 'forward': 

            # check what is directly in front of us
            # The idea is that we can speed up but if we did too much, rover had no chnce to react to obstacles
            # So I tuned it a bit based on the distance to an obstacle
            dead_ahead = Rover.nav_dists[np.vectorize(lambda t: t == 0, otypes =[np.bool])(Rover.nav_angles * 180/np.pi)]
            # print(dead_ahead )
            if len(dead_ahead[dead_ahead< 15]) == 0:
                Rover.max_vel = 0.1
            elif len(dead_ahead[dead_ahead > 100]) != 0:
                # print("GO")
                # print("GO")
                # print("GO")
                Rover.max_vel = 3
            else:
                # print("CARE")
                Rover.max_vel = 1
            
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.percentile(Rover.nav_angles * 180/np.pi, 70), -15, 15)
                # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
        # if we want to close to the sample
        # It means that we see the sample and we expect to have clear path to it (sadly not alwyas true)
        elif Rover.mode == 'closing_to_sample':
            print("Getting closer")
            if Rover.seeing_sample == True:
                Rover.steer = Rover.sample_angle
                # If we are going in direction of the sample
                if Rover.sample_angle < 3 or Rover.sample_angle > -3:
                    if Rover.near_sample:
                        # stop
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                    elif Rover.vel > 0.5:
                        # coast
                        Rover.throttle = 0
                        Rover.brake = 0
                    elif Rover.vel < 0.5:
                        # speed up a bit to close in
                        Rover.throttle = 1
                        Rover.brake = 0
                # If we are deviating, slow down with expectation to gradually fix the angle
                # of approach
                else:
                    if Rover.vel > 0:
                        # Set throttle value to throttle setting
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
            else:
                Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
    

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

