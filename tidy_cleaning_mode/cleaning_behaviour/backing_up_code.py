#calculating the heading and the position for the robot to push it to the targeted wall
            WpX = 0
            WpY = 0
            if self.isgreen == True:
                #calculating the push vector for the green wall
                changeX =  self.selecBox[0][2]-self.green_wall[0]
                changeY =  self.selecBox[0][3]-self.green_wall[1]
                boxHeading = math.atan2(changeY,changeX)
        
                #calculating the co ordinates the robot should try to reach to push the box from
               
                if (round(self.selecBox[0][2],1)>=0.0 and round(self.selecBox[0][3],1)>=0.0): # x&y
                    WpX = self.selecBox[0][2] - (0.25*math.cos(math.radians(boxHeading)))
                    WpY = self.selecBox[0][3] + (0.25*math.sin(math.radians(boxHeading)))
                    
                elif (round(self.selecBox[0][2],1)<=0.0 and round(self.selecBox[0][3],1)>=0.0): #-x&y
                    WpX = self.selecBox[0][2] - (0.25*math.cos(math.radians(boxHeading)))
                    WpY = self.selecBox[0][3]- (0.25*math.sin(math.radians(boxHeading)))
                    
                elif (round(self.selecBox[0][2],1)>=0.0 and round(self.selecBox[0][3],1)<=0.0): # x&-y
                    WpX= self.selecBox[0][2] - (0.25*math.cos(math.radians(boxHeading)))
                    WpY = self.selecBox[0][3] + (0.25*math.sin(math.radians(boxHeading)))
                    
                elif (round(self.selecBox[0][2],1)<=0.0 and round(self.selecBox[0][3],1)<=0.0): # -x&-y
                    WpX = self.selecBox[0][2]- (0.25*math.cos(math.radians(boxHeading)))
                    WpY = self.selecBox[0][3]- (0.25*math.sin(math.radians(boxHeading)))
                #calculating the heading to the way point or pushing co ordinate
                self.waypoint[0] = WpX
                self.waypoint[1] = WpY

                changeX =  self.waypoint[0] -self.X
                changeY =  self.waypoint[1] -self.Y
                #print(self.waypoint)
                self.wp_heading = math.atan2(self.waypoint[1] -self.Y,self.waypoint[0] -self.X)

                #calculating the heading for the robot face the box
                changeX =  self.selecBox[0][2] -self.X
                changeY =  self.selecBox[0][3] -self.Y
                self.target_heading = math.atan2(self.selecBox[0][3] -self.waypoint[1],self.selecBox[0][2] -self.waypoint[0])

                #self.target[0] = self.selecBox[0][2] - 0.25*math.cos(self.target_heading+radians(self.yaw))
                #self.target[1] = self.selecBox[0][3] - 0.25*math.sin(self.target_heading+radians(self.yaw))
                
            else:
                
                boxHeading = math.atan2(self.selecBox[0][3]-self.red_wall[1],self.selecBox[0][2]-self.red_wall[0])
                
                
                #calculating the co ordinates the robot should try to reach to push the box from
                
                if (round(self.selecBox[0][2],1)>=0.0 and round(self.selecBox[0][3],1)>=0.0): # x&y
                    WpX = self.selecBox[0][2] + (0.25*math.cos(math.radians(boxHeading)))
                    WpY = self.selecBox[0][3] - (0.25*math.sin(math.radians(boxHeading)))
                    
                elif (round(self.selecBox[0][2],1)<=0.0 and round(self.selecBox[0][3],1)>=0.0): #-x&y
                    WpX = self.selecBox[0][2] + (0.25*math.cos(math.radians(boxHeading)))
                    WpY = self.selecBox[0][3]+ (0.25*math.sin(math.radians(boxHeading)))
                    
                elif (round(self.selecBox[0][2],1)>=0.0 and round(self.selecBox[0][3],1)<=0.0): # x&-y
                    WpX= self.selecBox[0][2] + (0.25*math.cos(math.radians(boxHeading)))
                    WpY = self.selecBox[0][3] - (0.25*math.sin(math.radians(boxHeading)))
                    
                elif (round(self.selecBox[0][2],1)<=0.0 and round(self.selecBox[0][3],1)<=0.0): # -x&-y
                    WpX = self.selecBox[0][2]+ (0.25*math.cos(math.radians(boxHeading)))
                    WpY = self.selecBox[0][3]+ (0.25*math.sin(math.radians(boxHeading)))
                   
                
                #calculating the heading to the way point or pushing co ordinate
                self.waypoint[0] = WpX
                self.waypoint[1] = WpY

                changeX =  self.waypoint[0] -self.X
                changeY =  self.waypoint[1] -self.Y
                
                self.wp_heading = math.atan2(self.waypoint[1] -self.Y,self.waypoint[0] -self.X)
                #print(self.waypoint," ",self.wp_heading)
                #calculating the heading for the robot face the box
                #changeX =  
                #changeY =  
                self.target_heading = math.atan2(self.selecBox[0][3] -self.waypoint[1],self.selecBox[0][2] -self.waypoint[0])