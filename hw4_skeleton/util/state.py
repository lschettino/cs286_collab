from globals import globals
import numpy as np
import sys
from itertools import compress

class state:

    def __init__(self, line=None, num_agents=None):
        #example line="0,2;0,3;1,3;0,3;0;0;0;0;4,2-1,2;1,0-3,0;0,0-4,3"
        self.g = globals()
        if num_agents is not None:
            self.g.m=num_agents
        self.time = 0
        if line is None:
            self.agent_locations = [[np.random.randint(self.g.num_x), np.random.randint(self.g.num_y)] for ell in range(self.g.m)]
            self.time_left_in_current_trip = [0 for ell in range(self.g.m)]
            self.outstanding_requests = []  # a list of <request_time, pickup_x, pickup_y, dropoff_x, dropoff_y>
            #self.outstanding_requests = [(np.random.randint(self.g.num_x), np.random.randint(self.g.num_y),
            #                              np.random.randint(self.g.num_x), np.random.randint(self.g.num_y)) for s in range(2)]
        else:
            line_elements = line.split(';')
            location_elements = line_elements[ : self.g.m]
            time_left_elements = line_elements[self.g.m : 2 * self.g.m]
            outstanding_requests_elements = line_elements[2 * self.g.m : ]
            #print(location_elements)
            #print(time_left_elements)
            #print(outstanding_requests_elements)
            self.agent_locations = [[int(location_elements[ell].split(',')[0]), int(location_elements[ell].split(',')[1])] for ell in range(self.g.m)]
            self.time_left_in_current_trip = [int(time_left_elements[ell]) for ell in range(self.g.m)]
            self.outstanding_requests = []
            #print(outstanding_requests_elements, len(outstanding_requests_elements))
            for s in range(len(outstanding_requests_elements)):
                outstanding_request = outstanding_requests_elements[s].split('-')
                self.outstanding_requests.append((int(outstanding_request[0].split(',')[0]), int(outstanding_request[0].split(',')[1]), #pickup_x, pickup_y
                                                 int(outstanding_request[1].split(',')[0]), int(outstanding_request[1].split(',')[1]))) #dropoff_x, dropoff_y

    def print_state(self):
        #print(self.agent_locations)
        #print(self.time_left_in_current_trip)
        #print(self.outstanding_requests)
        state_string = ";".join([str(location[0])+","+str(location[1]) for location in self.agent_locations])
        state_string += ";" + ";".join([str(time_left) for time_left in self.time_left_in_current_trip])
        if(len(self.outstanding_requests)>0):
            state_string += ";" + ";".join([str(request[0])+","+str(request[1])+"-"+str(request[2])+","+str(request[3]) for request in self.outstanding_requests])
        return state_string

    def available_control_agent(self, ell):
        available_control_agent_ = []
        # print('Inside available_control_agent('+str(ell)+')')
        available_control_agent_.append(0)
        if self.time_left_in_current_trip[ell] == 0:
            agent_x, agent_y = self.agent_locations[ell]
            if (agent_x > 0):
                available_control_agent_.append(1)
            if (agent_x < self.g.num_x - 1):
                available_control_agent_.append(2)
            if (agent_y > 0):
                available_control_agent_.append(3)
            if (agent_y < self.g.num_y - 1):
                available_control_agent_.append(4)
        if self.time_left_in_current_trip[ell] == 0:
            requests = [5 + request_id_details[0] for request_id_details in
                        list(filter(lambda x: x[1][0] == agent_x and x[1][1] == agent_y, enumerate(self.outstanding_requests)))]
            available_control_agent_.extend(requests)

        return available_control_agent_

    def sample_new_requests(self):
        new_requests = []
        number_new_request = np.random.choice(np.arange(3), p=[0.5, 0.3, 0.2])
        for new_request in range(number_new_request):
            pickup_locations = list(self.g.pickup_distribution.keys())
            pickup_prob = list(self.g.pickup_distribution.values())
            pickup_location_id = np.random.choice(np.arange(len(pickup_locations)), p=pickup_prob)
            pickup_location_ = pickup_locations[pickup_location_id]

            dropoff_locations = list(self.g.dropoff_distribution.keys())
            dropoff_prob = list(self.g.dropoff_distribution.values())
            dropoff_location_id = np.random.choice(np.arange(len(dropoff_locations)), p=dropoff_prob)
            dropoff_location_ = dropoff_locations[dropoff_location_id]
            new_requests.append((pickup_location_[0], pickup_location_[1], dropoff_location_[0], dropoff_location_[1]))
        return new_requests

    def next_state(self, control, known_requests=None):
        # print('Inside next_state with control:', control)
        stage_cost = len(self.outstanding_requests)
        # print('stage_cost ', stage_cost)
        mask = [True for s in range(stage_cost)]
        for ell in range(self.g.m):
            if (self.time_left_in_current_trip[ell] > 0):
                self.time_left_in_current_trip[ell] -= 1

            elif self.time_left_in_current_trip[ell] == 0 and control[ell] < 5:
                if control[ell] == 1:
                    self.agent_locations[ell][0] -= 1
                elif control[ell] == 2:
                    self.agent_locations[ell][0] += 1
                elif control[ell] == 3:
                    self.agent_locations[ell][1] -= 1
                elif control[ell] == 4:
                    self.agent_locations[ell][1] += 1

            elif self.time_left_in_current_trip[ell] == 0 and control[ell] >= 5:
                # print('control.index',control.index(control[ell]))
                if (control.index(control[ell]) == ell):
                    pickup_location = self.outstanding_requests[control[ell] - 5][0:2]
                    dropoff_location = self.outstanding_requests[control[ell] - 5][2:4]
                    self.time_left_in_current_trip[ell] = self.g.manhattan_distance(pickup_location, dropoff_location)
                    self.agent_locations[ell] = [dropoff_location[0], dropoff_location[1]]
                    mask[control[ell] - 5] = False

        self.outstanding_requests = list(compress(self.outstanding_requests, mask))

        if known_requests is None:
            # Sample new request(s)
            new_requests = self.sample_new_requests()
        else:
            new_requests = known_requests
        self.outstanding_requests.extend(new_requests)

        self.time += 1
        return stage_cost

    def terminal_cost(self):
        dist = 0
        for request in self.outstanding_requests:
            for ell in range(self.g.m):
                dist += self.g.manhattan_distance((request[0],request[1]), (self.agent_locations[ell][0],self.agent_locations[ell][1]))
        return int(dist / 4)