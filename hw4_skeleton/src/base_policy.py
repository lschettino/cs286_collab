class base_policy:
    def get_control(self, taxi_state_object):
        #print('get_control() base policy')
        control = []
        for ell in range(taxi_state_object.g.m):
            control.append(self.get_base_policy_control_component(taxi_state_object, ell))
        return control

    def next_direction(self, location1, location2):
        dir = 0
        # Return the direction of motion 0:stay, 1:left, 2:right, 3:up, 4:down based on the minimum manhattan distance between location1, location2
        # You can give horizontal motion more presidence over vertical motion
        # for example location1 = (1, 1), location2 = (2, 3), the fuction returns 2.
        ################################# Begin your code ###############################

        ################################# End your code ###############################
        return dir

    def get_base_policy_control_component(self, taxi_state_object, ell):
        control_component = None
        # If a request's pickup location is the same as agent ell's location and the taxi is availble, return the 5+index of the request in the outstanding_requests list
        # Else if the taxi is available, return the direction of motion using the taxi ell's location and the nearest request's pickup location
        ################################# Begin your code ###############################

        ################################# End your code ###############################
        if control_component is None:
            return 0
        return control_component

