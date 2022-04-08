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

        # x = math.abs(location1[0] - location2[0])
        # y = math.abs(location1[1] - location2[1])
        if location1[0] - location2[0] > 0:
            dir = 1
        elif location1[0] - location2[0] < 0:
            dir = 2
        else:
            if location1[1] - location2[1] > 0:
                dir = 3
            elif location1[1] - location2[1] < 0:
                dir = 4 
            else: 
                dir = 0


        ################################# End your code ###############################
        return dir

    def get_base_policy_control_component(self, taxi_state_object, ell):
        control_component = None
        # If a request's pickup location is the same as agent ell's location and the taxi is availble, return the 5+index of the request in the outstanding_requests list
        # Else if the taxi is available, return the direction of motion using the taxi ell's location and the nearest request's pickup location
        ################################# Begin your code ###############################

        if len(taxi_state_object.outstanding_requests) == 0 or taxi_state_object.time_left_in_current_trip[ell] > 0:
            return 0

        ell_location = taxi_state_object.agent_locations[ell]

        request_ind = 0
        smallest_distance = float('inf')
        ind = 0
        for pickup_x, pickup_y, _, _  in taxi_state_object.outstanding_requests:
            dist = abs(ell_location[0] - pickup_x) + abs(ell_location[1] - pickup_y)
            if dist < smallest_distance:
                smallest_distance = dist
                request_ind = ind
            ind += 1

        if smallest_distance == 0:
            control_component = 5 + request_ind
        else:
            control_component = self.next_direction(ell_location, taxi_state_object.outstanding_requests[request_ind])

        ################################# End your code ###############################
        if control_component is None:
            return 0
        return control_component

