import numpy as np
import math, sys
import dynamic_prog
import matplotlib.pyplot as plt

#orientation = [[135, 90, 45],
#		[180, 0, 0],
#		[225, 270, 315]]


orientation = [[225, 270, 315],
		[180, 0, 0],
		[135, 90, 45]]

def dist_to_goal(Rover, goal):
	x, y = Rover.pos
	dist = math.sqrt((x-goal[0])**2 + (y-goal[1])**2)
	return dist

def world_to_obs(WorldMap):
	obs = np.full_like(WorldMap[:,:,0], 1)
	for x in range(len(WorldMap)):
		for y in range(len(WorldMap[x])):
			if WorldMap[x,y,2] > 1:
				obs[x,y] = 0
	return obs


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    if Rover.step != 0:
    	Rover.step -= 1
    else:
    	Rover.steer = 0
    #Rover.test -= 1
    #if Rover.test >= 0:
    #	print(Rover.test)
    if Rover.initial_pos == None:
    	Rover.initial_pos = Rover.pos
    #if True and Rover.test <= 0:
    if Rover.samples_collected == Rover.samples_to_find:
    	x, y = int(Rover.pos[0]), int(Rover.pos[1])
    	if Rover.ret_step != 0:
    		Rover.ret_step -= 1
    	if Rover.skip != 0:
    		Rover.skip -= 1


    	print("returning")
    
    	if abs(Rover.vel) !=  0. and Rover.ret_step == 0:
    		print("stopper for return")
    		Rover.brake = Rover.brake_set
    		Rover.throttle = 0 
    		return Rover
    	if abs(Rover.vel) == 0:
    		Rover.brake = 0.
    	if Rover.route_map is None or Rover.route_map[y][x] == 9999:
    		way_map = world_to_obs(Rover.worldmap)
    		#np.savetxt("test2.txt", way_map, fmt='%d')
    		Rover.route_map = dynamic_prog.dynamic_path_finder(way_map, goal=(int(Rover.initial_pos[1]), int(Rover.initial_pos[0])))
    		#np.savetxt("test.txt", Rover.route_map, fmt='%4d')
    		#for x in range(len(Rover.worldmap)):
    		#	for y in range(len(Rover.worldmap[x])):
    		#		if Rover.worldmap[x,y,2] > 1:
    		#			for i in range(-1, 2):
    		#				for j in range(-1, 2):
    		#					if x+i >= 0 and x+i < len(Rover.worldmap) and y+j >= 0 and y+j < len(Rover.worldmap[x]):
    		#						if Rover.worldmap[x+i,y+j,0] > 0:
    		#							Rover.route_map[x,y] += 2


    		
    	if dist_to_goal(Rover, (int(Rover.initial_pos[0]), int(Rover.initial_pos[1]))) < 5:
    		Rover.brake = Rover.brake_set
    		Rover.throttle = 0
    		print("reached")
    	else:
    		print("getting back")
    		min_val = Rover.route_map[y, x]
    		rotate = 0
    		dodge_spin = 0
    		xj, yi = [0], [0] 
    		for i in range(-1, 2):
    			for j in range(-1, 2):
    				if y+i >= 0 and y+i < len(Rover.route_map) and x+j >= 0 and x+j < len(Rover.route_map[y]):
    					#print(Rover.route_map[y+i, x+j], end=" ")
    					if i == 0 and j == 0:
    						continue 
    					if Rover.route_map[y+i, x+j] < min_val:
    						xj, yi = [j], [i]
    						rotate = orientation[i+1][j+1] 
    						min_val = Rover.route_map[y+i, x+j]
    					elif Rover.route_map[y+i, x+j] == min_val:
    						xj.append(j)
    						yi.append(i)
    						rotate = ((rotate * (len(xj) - 1)) + orientation[i+1][j+1])/len(xj)
    					
    			print("\n")
    		print(xj, yi, rotate)

    		#min_dodge = 9999
    		#for i in range(-1, 2):
    		#	for j in range(-1, 2):
    		#		if y+yi+i >= 0 and y+yi+i < len(Rover.worldmap) and x+xj+j >= 0 and x+xj+j < len(Rover.worldmap[y]):
	    	
	    	if abs(rotate - Rover.yaw) > 3 and Rover.ret_step == 0:
	    		if Rover.skip == 0:
		    		Rover.steer = np.clip(rotate - Rover.yaw, -15, 15)
		    		Rover.skip = 3
	    	else:
	    		Rover.steer = 0
	    		p = np.searchsorted( Rover.obs_angles, Rover.steer)
	    		Rover.obs_dists = [di for _,di in sorted(zip(Rover.obs_angles, Rover.obs_dists))]
	    		Rover.obs_angles.sort()
	    		for i in range(-20, 21):
	    			if p+i >= 0 and p+i < len(Rover.obs_dists):
	    				if Rover.obs_dists[p+i] < 15:
	    					Rover.nav_dists = [di for _,di in sorted(zip(Rover.nav_angles, Rover.nav_dists))]
	    					Rover.nav_angles.sort()
	    					Rover.steer = np.average(Rover.nav_angles, weights=Rover.nav_dists) *  180/np.pi
	    					

    			Rover.throttle = 0.
    			if Rover.steer == 0:
    				Rover.throttle = 0.5
    			Rover.brake = 0
    			if Rover.ret_step == 0:
    				Rover.ret_step = 10
    			


    		return Rover 
    #return Rover


    if Rover.steer_angle_count == None:
    	Rover.steer_angle_count = 0

    if Rover.stuck_maneuver == 1:
    	if not Rover.left_stuck:
    		Rover.rock_refound = 0

    		Rover.steer = np.clip(np.average(Rover.nav_angles, weights=Rover.nav_dists) *  180/np.pi , -15, 15)
    		Rover.step = 1
    		p = np.searchsorted( Rover.obs_angles, Rover.steer)
    		Rover.obs_dists = [di for _,di in sorted(zip(Rover.obs_angles, Rover.obs_dists))]
    		Rover.obs_angles.sort()
    		for i in range(-20, 21):
    			if p+i >= 0 and p+i < len(Rover.obs_dists):
    				if Rover.obs_dists[p+i] < 4:
    					Rover.throttle = 0
    					Rover.brake = Rover.brake_set
    					if Rover.steer != 0:
    						Rover.last_steer = Rover.steer
    					#Rover.steer = 0
    					Rover.mode = 'stop'
    		go = False
    		if Rover.mode == 'stop':
    			go = True

    		while go:
    			p = np.searchsorted(Rover.obs_angles, Rover.steer)
    			go = False
    			for i in range(-20, 21):
    				if p+i >= 0 and p+i < len(Rover.obs_dists):
    					if Rover.obs_dists[p+i] < 5:
    						if Rover.last_steer < 0:
    							Rover.steer += 1
    						else:
    							Rover.steer -= 1
    						go = True
    						break
    		Rover.step = 1
    		Rover.mode = 'forward'
    		Rover.throttle = 0.15
    		Rover.brake = 0
    		if abs(Rover.vel) > 0.05 :
    			Rover.left_stuck = True

    		print("dodge attempt")
    		return Rover
    	else:
    		if abs(Rover.total_change) > 360:
    			Rover.stuck_maneuver = 0
    		if abs(Rover.vel) != 0:
    			Rover.brake = Rover.brake_set
    			Rover.throttle = 0
    		else:
    			if Rover.rock_refound >= 3:
    				Rover.stuck_maneuver = 0
    			else:
    				if Rover.rock_angles is not None:
	    				Rover.rock_refound += 1
	    				#print("\n\n\n",len(Rover.rock_angles),"\n\n\n")
	    				if len(Rover.rock_angles) > 15:
	    					Rover.steer = np.clip(np.average(Rover.nav_angles, weights=Rover.nav_dists) *  180/np.pi, -1, 1)
	    					Rover.total_change += Rover.steer
	    				else:
	    					Rover.steer = np.clip(np.average(Rover.nav_angles, weights=Rover.nav_dists) *  180/np.pi, -3, 3)
	    					Rover.total_change += Rover.steer
	    			else:
	    				Rover.rock_refound = 0
	    				Rover.steer = -10
	    				Rover.total_change += Rover.steer

                    	

    


    # Example:
    # Check if we have vision data to make decisions with
    Rover.nav_dists = [di for _,di in sorted(zip(Rover.nav_angles, Rover.nav_dists))]
    Rover.nav_angles.sort()
    if np.sum(Rover.nav_dists) != 0: 
	    avg = np.average(Rover.nav_angles, weights=Rover.nav_dists) *  180/np.pi
	    if abs(avg) < 10:
	    	loc = np.searchsorted(Rover.nav_angles, avg)

	    	dists1 = Rover.nav_dists[:loc]
	    	dists2 = Rover.nav_dists[loc:]
	    	if np.sum(dists1) != 0 and np.sum(dists2) != 0:
		    	avg1 = np.average(Rover.nav_angles[:loc], weights=dists1) *  180/np.pi
		    	avg2 = np.average(Rover.nav_angles[loc:], weights=dists2) *  180/np.pi
		    	if abs(avg1) > 10 and abs(avg2) > 10:
		    		p1 = np.searchsorted(Rover.nav_angles[:loc], avg1)
		    		p2 = np.searchsorted(Rover.nav_angles[loc:], avg2)
		    		max1, max2 = 0, 0
		    		amax1, amax2 = 0, 0
		    		for i in range(-20, 21):
		    			if p1+i >= 0 and p1+i < len(Rover.nav_dists[:loc]):
		    				if Rover.nav_dists[:loc][p1+i] > max1:
		    					max1 = Rover.nav_dists[:loc][p1+i]
		    					amax1 = Rover.nav_angles[:loc][p1+i] * 180/np.pi
		    					
		    			if p2+i >= 0 and p2+i < len(Rover.nav_dists[loc:]):
		    				if Rover.nav_dists[loc:][p2+i] > max2:
		    					max2 = Rover.nav_dists[loc:][p2+i]
		    					amax2 = Rover.nav_angles[loc:][p2+i] * 180/np.pi
		    					
		    		print("maybe two sides to go:  ", amax1, max1, amax2, max2)
    
    
    temp = Rover.steer
    
    
    if Rover.near_sample and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.brake = Rover.brake_set
        Rover.throttle =  0
        Rover.stuck = 0
        print("initiate rock pick up")
        return Rover
    
    if Rover.rock_angles is not None:
    	Rover.rock_step += 1
    	#print("\n\n\n",len(Rover.rock_angles),"\n\n\n")
    	if len(Rover.rock_angles) > 10 and Rover.stuck < 15:
    		#print(len(Rover.rock_angles))
    		if Rover.rock_step >= 0 and Rover.rock_step <= 3 and abs(Rover.vel) > 0.7:
    			Rover.brake = Rover.brake_set
    			Rover.throttle = 0
    			print("rocks! too fast, slow down")
    			return Rover
    		if abs(Rover.vel) < 0.05:
    			Rover.stuck += 1
    			Rover.brake = 0
    			Rover.throttle = Rover.throttle_set
    			print("maybe stuck, increase speed")
    			return Rover

    		if Rover.rock_step > 5 and abs(Rover.vel) > 0.5:
    			Rover.stuck = 0
    			Rover.rock_step = 0


    		Rover.steer = Rover.rock_angles[np.argmin(Rover.rock_dists)] * 180/ np.pi
    		p = np.searchsorted( Rover.obs_angles, Rover.steer)
    		Rover.obs_dists = [di for _,di in sorted(zip(Rover.obs_angles, Rover.obs_dists))]
    		Rover.obs_angles.sort()
    		for i in range(-20, 21):
    			if p+i >= 0 and p+i < len(Rover.obs_dists):
                		if Rover.obs_dists[p+i] < 5:
                			Rover.throttle = 0
                			Rover.brake = Rover.brake_set
                			Rover.stuck = 0
                			Rover.stuck_maneuver = 1 
                			Rover.left_stuck = False
                			Rover.rock_refound = 0
                			Rover.total_change = 0
                			print("rock obs")
                			return Rover
    		Rover.step = 2
    		Rover.mode = 'forward'
    		Rover.brake = 0
    		Rover.throttle = 0.07
    		if abs(Rover.steer) > 10:
    			Rover.throttle = 0
    			Rover.stuck -= 1
    		if Rover.rock_dists[np.argmin(Rover.rock_dists)] <= 4 :
    			Rover.brake = Rover.brake_set
    			Rover.throttle = 0
    			Rover.mode = 'stop'
    			Rover.stuck = 0
    		print("rocks!")
    		return Rover
  
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                Rover.rock_step = 0
                if abs(Rover.vel) < 0.05 :
                    Rover.stuck += 1
                    # Set brake to stored brake value
                    Rover.brake = 0
                    if Rover.steer != 0:
                    	Rover.last_steer = Rover.steer
                    Rover.steer = 0
                    Rover.throttle = Rover.throttle_set

                    if Rover.stuck >= 15 and Rover.stuck < 20:
                    	Rover.throttle = 0
                    	Rover.brake = 0
                    	Rover.steer = np.average(Rover.nav_angles, weights=Rover.nav_dists) *  180/np.pi
                    	if Rover.stuck in [16, 17, 18] and abs(np.average(Rover.nav_angles, weights=Rover.nav_dists) *  180/np.pi) < 8 :
                    		Rover.throttle = -0.3
                    		Rover.brake = 0
                    		Rover.mode = 'forward'
                    		Rover.steer = -np.clip(np.average(Rover.nav_angles, weights=Rover.nav_dists) *  180/np.pi , -10, 10)
                    	#Rover.stuck = 0
                    elif Rover.stuck >=20:
                    	Rover.mode = 'stop'
                    	Rover.brake = Rover.brake_set
                    	Rover.throttle = 0
                    print("stuck", Rover.stuck)
                    return Rover
                elif abs(Rover.vel) < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                    Rover.stuck = 0
                else: # Else coast
                    Rover.throttle = 0
                    Rover.stuck = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                if Rover.step == 0:
                	Rover.steer = np.clip(np.average(Rover.nav_angles, weights=Rover.nav_dists) *  180/np.pi , -15, 15)
                	Rover.step = 1
                	p = np.searchsorted( Rover.obs_angles, Rover.steer)
                	#go = False
                	Rover.obs_dists = [di for _, di in sorted(zip(Rover.obs_angles, Rover.obs_dists))]
                	Rover.obs_angles.sort()
                	for i in range(-20, 21):
                		if p+i >= 0 and p+i < len(Rover.obs_dists):
	                		if Rover.obs_dists[p+i] < 4:
	                			Rover.throttle = 0
	                			Rover.brake = Rover.brake_set
	                			if Rover.steer != 0:
	                    				Rover.last_steer = Rover.steer
	                			#Rover.steer = 0
	                			Rover.mode = 'stop'
                go = False
                if Rover.mode == "stop":
                	go = True
                
                while go:
                	p = np.searchsorted(Rover.obs_angles, Rover.steer)
                	go = False
                	for i in range(-20, 21):
                		if p+i >= 0 and p+i < len(Rover.obs_dists):
                			if Rover.obs_dists[p+i] < 10:
                				if Rover.last_steer < 0:
                					Rover.steer += 1
                				else:
                					Rover.steer -= 1
                				go = True
                Rover.step = 1
                Rover.mode = 'forward'
                Rover.throttle = 0.1
                Rover.brake = 0
                if abs(Rover.steer) > 10 and abs(Rover.vel) > 0.5:
                	Rover.throttle = 0
                print("move it")
                
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    if Rover.steer != 0:
                    	Rover.last_steer = Rover.steer
                    Rover.steer = 0
                    Rover.mode = 'stop'
                    print("stopper")

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            Rover.stuck = 0
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                if Rover.steer != 0:
                	Rover.last_steer = Rover.steer
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -5 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    if Rover.step == 0:
                    	Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    	Rover.step = 1
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        if Rover.steer != 0:
            Rover.last_steer = Rover.steer
        Rover.steer = 0
        Rover.brake = 0
        print("else something else")
        
    # If in a state where want to pickup a rock send pickup command

    if temp == Rover.steer:
    	Rover.steer_angle_count +=1
    else:
    	Rover.steer_angle_count = 0

    if Rover.steer_angle_count >= 100:
    	if Rover.vel == 0:
    		Rover.steer_angle_count = 0
    		Rover.mode = 'stop'
    	Rover.brake = 0
    	Rover.throttle = 0
    	Rover.steer = -15


    return Rover

