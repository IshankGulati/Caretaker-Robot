import cv2
import numpy as np
import math
import serial
import heapq
import time


'''
Function: get_perspective_image()
            helps to get an image of arena
'''
def get_perspective_image(frame):
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    lower = np.array([0, 0, 0]) #black color mask
    upper = np.array([120, 120, 120])
    mask = cv2.inRange(frame, lower, upper)
    
    ret,thresh1 = cv2.threshold(mask,127,255,cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(frame,contours,-1,(0,255,0),3)
    biggest = 0
    max_area = 0
    min_size = thresh1.size/4
    index1 = 0
    for i in contours:
        area = cv2.contourArea(i)
        if area > 10000:
            peri = cv2.arcLength(i,True)
        if area > max_area: 
            biggest = index1
            max_area = area
        index1 = index1 + 1
    approx = cv2.approxPolyDP(contours[biggest],0.05*peri,True)
    #drawing the biggest polyline
    cv2.polylines(frame, [approx], True, (0,255,0), 3)
    x1 = approx[0][0][0]
    y1 = approx[0][0][1]
    x2 = approx[1][0][0]
    y2 = approx[1][0][1]
    x3 = approx[3][0][0]
    y3 = approx[3][0][1]
    x4 = approx[2][0][0]
    y4 = approx[2][0][1]

    print x1, y1
    print x2, y2
    print x3, y3
    print x4, y4
    
    #points remapped from source image from camera
    #to cropped image try to match x1, y1,.... to the respective near values
    pts1 = np.float32([[x2,y2],[x4,y4],[x1,y1],[x3,y3]]) 
    pts2 = np.float32([[0,0],[0,480],[540,0],[540,480]])
    persM = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(frame,persM,(540,480))
    return dst

ser = serial.Serial(
    port='COM7',
    baudrate=9600,
    timeout=0,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

#assign threshold level for respective colours
low_green = [73,100,50]
high_green = [97,255,255]
low_red = [0,100,100]
high_red = [20,255,255]
low_yellow = [14,100,100]
high_yellow = [40,255,255]
low_blue = [94,100,100]
high_blue = [119,255,255]
low_violet = [129,10,100]
high_violet = [169,255,255]
low_orange = [0,100,100]
high_orange = [20,255,255]

rows=5
columns=5

obstacles,supply_b,table_b,supply_r,table_r,supply_y,table_y,o,v = ([]for i in range(9))
    
class ArenaMap(object):
    '''
    Function: mask()
    input: image, low level threshold, high level threshold
    output: image
    Logic: mask the image and turns other pixel into black which have their hsv values outside the band of threshold
    '''
    def mask(self,img, hsv_low, hsv_high):
        #converts bgr to hsv color channel
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # Convert the parameters into a form that OpenCV can understand
        lower_mask = np.array(hsv_low)
        upper_mask = np.array(hsv_high)
        #mask the image on the basis of hsv range
        mask = cv2.inRange(hsv, lower_mask, upper_mask)
        #logical Anding to convert the masked part to its original color
        img = cv2.bitwise_and(img,img, mask= mask)
        return img

    '''
    Function: find_countours()
    input: image, id
    output: depending on the value of id we can have following outputs
            *co-ordinate of provisions of respective colour on supply and table side both
            *co-ordinate of markers mounted on robot to detect orientation of robot
            *co-ordinate of green walls in the corridor
    Logic: by calculating the centroid of countours we can find the co-orinate
    '''
    def find_contours(self,img,id):
        #finding countours
        temp,temp_s,temp_m  = ([]for i in range(3))
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,30,255,cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img,contours,-1,(0,0,255),3)
        for i in range(0,len(contours)):
            #finding area of respective countours
            area = cv2.contourArea(contours[i])
            if area > 1800:
                #it is our green obstacles in the corridor
                flag =1
                M = cv2.moments(contours[i])
                cx = int(M['m10']/M['m00']) + w/5
                cy = int(M['m01']/M['m00']) 
                temp.append((cx,cy))
            elif area > 1000:
                #provisions are detected
                flag = 0
                M = cv2.moments(contours[i])
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if cx > 300:
                    temp_m.append((cx, cy))
                elif cx < 230 :
                    temp_s.append((cx, cy))
            elif area > 500 and id != 0:
                #markers mounted on robot are detected
                flag =1
                M = cv2.moments(contours[i])
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00']) 
                temp.append((cx,cy))
        if flag == 0:
            #print temp_s,temp_m
            return temp_s,temp_m
        elif flag == 1:
            return temp

    '''
    Function: grid_map()
    input: co-ordinates of obstacles, start and end positions
    output: matrix of 5x5 having 0 for path that can be followed; 1= obstacles; 2=start; 3=end;
    Logic: comparing the co-orinate value and assigning that grid value accordingly
    '''
    def grid_map(self,obstacles,start,end):
        grid=np.zeros((rows,columns))
        #mapping of arena in grid of 5x5
        #obstacles=1
        #start=2
        #end=3
        for i in range (0,4): #no. of green obstacles = 4
            if obstacles[i][0] / 270 == 0:
                if obstacles[i][1] / 120 == 0:
                    grid[0][1] = 1
                elif obstacles[i][1] / 240 == 0:
                    grid[1][1] = 1
                elif obstacles[i][1] / 360 == 0:
                    grid[2][1] = 1
                elif obstacles[i][1] / 480 == 0:
                    grid[3][1] = 1
            else:
                if obstacles[i][1] / 120 == 0:
                    grid[0][3] = 1
                elif obstacles[i][1] / 240 == 0:
                    grid[1][3] = 1
                elif obstacles[i][1] / 360 == 0:
                    grid[2][3] = 1
                elif obstacles[i][1] / 480 == 0:
                    grid[3][3] = 1
        if start[0]<100 and start[1]<160:
            grid[0][0] = 2
        elif start[0]<100 and start[1]<320:
            grid[2][0] = 2
        elif start[0]<100 and start[1]<480:
            grid[3][0] = 2
        elif start[0]>450 and start[1]<160:
            grid[0][4] = 2
        elif start[0]>450 and start[1]<320:
            grid[2][4] = 2
        elif start[0]>450 and start[1]<480:
            grid[3][4] = 2
        elif start[1]<120:
            grid[0][2] = 2
        elif start[1]<360:
            grid[2][2] = 2
        elif start[1]<480:
            grid[3][2] = 2
         
            
        if end[0]<100 and end[1]<160:
            grid[0][0] = 3
        elif end[0]<100 and end[1]<320:
            grid[2][0] = 3
        elif end[0]<100 and end[1]<480:
            grid[3][0] = 3
        elif end[0]>450 and end[1]<160:
            grid[0][4] = 3
        elif end[0]>450 and end[1]<320:
            grid[2][4] = 3
        elif end[0]>450 and end[1]<480:
            grid[3][4] = 3
        elif end[1]<120:
            grid[0][2] = 3
        elif end[1]<360:
            grid[2][2] = 3
        elif end[1]<480:
            grid[3][2] = 3    
        #assigning last row as obstacles to maintain 5x5 matrix
        grid[4][0]=grid[4][1]=grid[4][2]=grid[4][3]=grid[4][4]=1
        return grid 

'''
Function: orientation()
input: co-ordinates of marker mounted on robot
output: orientaion: north='n'; south='s'; east='e'; west='w'
Logic: by comparing the distance of front and back marker from top and left side of the image we can have the idea about orientation of robot
'''
def orientation(v,o):
    #to find the orientation of robot on the basis of marker mounted
    if v[1] - o[1] > 20:
        #if distance of back marker is greater than front marker from top side of image than it heads towards north
        orient = 'n'
    elif o[1] - v[1] > 20:
        #if distance of back marker is smaller than front marker from top side of image than it heads towards south
        orient = 's'
    elif v[0] - o[0] > 20:
        #if distance of back marker is greater than front marker from left side of image than it heads towards west
        orient = 'w'
    elif o[0] - v[0] > 20:
        #if distance of back marker is smaller than front marker from left side of image than it heads towards east
        orient = 'e'
    return orient

'''
Function: orient_correct()
input: current orientation of robot; desired orientation of robot and its position
output: none
Logic: on comparision between current and desired orientation robot is turned 90 or 180 degree
'''
def orient_correct(current_orient,desired_orient,position):
    #function to change the orientation of robot to desired orientation 
    global orient
    if current_orient == 'e' and desired_orient == 'w':
        ser.write("\x15")
        #180 degree turn
        time.sleep(3)
    if current_orient == 'w' and desired_orient == 'e':
        ser.write("\x13")
        #180 degree turn
        time.sleep(3)
    if current_orient == 'n' and desired_orient == 's':
        ser.write("\x15")
        #180 degree turn
        time.sleep(3)
    if current_orient == 's' and desired_orient == 'n':
        ser.write("\x15")
        #180 degree turn
        time.sleep(3)
    if current_orient == 'e' and desired_orient == 'n':
        ser.write("\x04")
        #90 degree turn left
    if current_orient == 'e' and desired_orient == 's':
        ser.write("\x03")
        #90 degree turn right
    if current_orient == 'w' and desired_orient == 'n':
        ser.write("\x03")
        #90 degree turn right
    if current_orient == 'w' and desired_orient == 's':
        ser.write("\x04")
        #90 degree turn left
    if current_orient == 'n' and desired_orient == 'e':
        ser.write("\x03")
        #90 degree turn left
    if current_orient == 'n' and desired_orient == 'w':
        ser.write("\x04")
        #90 degree turn right
    if current_orient == 's' and desired_orient == 'e':
        ser.write("\x04")
        #90 degree turn right
    if current_orient == 's' and desired_orient == 'w':
        ser.write("\x03")
        #90 degree turn left
    time.sleep(3)
    orient = desired_orient
        
'''
Function: send()
input: index of grid matrix(current and next index)
output: none
Logic: robot is moved by checking its orientation and the change in index(if x-axis changes then we have to move either east(postive change) or west(negative change);
        if y-axis changes then we have to move either north(negative change) or south(positive change)
'''
def send(data1,data2):
    global orient
    if data1[0] > data2[0] and data1[1] == data2[1]:
        #movement in west direction
        if orient != 'w':
            #correct the orientation
            orient_correct(orient,'w',data1)
        if data2[0] == 2 or data2[0] == 4:
            if data2[1] ==2:
                ser.write("\x16")
            else:
                ser.write("\x05")
        else:
            ser.write("\x05")
        print 'forward'
        time.sleep(2.5)
    elif data1[0] < data2[0] and data1[1] == data2[1]:
        #movement in east direction
        if orient != 'e':
            #correct the orientation
            orient_correct(orient,'e',data1)
        if data2[0] == 2 or data2[0] == 4:
            if data2[1] ==2:
                ser.write("\x16")
            else:
                ser.write("\x05")
        else:
            ser.write("\x05")
        print 'forward'
        time.sleep(2.5)
    elif data1[0] == data2[0] and data1[1] > data2[1]:
        #movement in north direction
        if orient != 'n':
            #correct the orientation
            orient_correct(orient,'n',data1)
        if data2[0]==3:
            ser.write("\x01")
        else:
            ser.write("\x14")
        print 'forward'
        time.sleep(2.5)
    elif data1[0] == data2[0] and data1[1] < data2[1]:
        #movement in south direction
        if orient != 's':
            #correct the orientation
            orient_correct(orient,'s',data1)
        if data2[0]==3:
            ser.write("\x01")
        else:
            ser.write("\x14")
        print 'forward'
        time.sleep(2.5)
    elif data1[0] == data2[0] and data1[1] == data2[1]:
        #no movement
        pass

'''
Function: pick_up()
input: initial position, path, path_length, colour of provision robot is going to pick
output: end position of robot
Logic: traverse the path and glow the desired led
'''
def pick_up(initial,path,length,color):
    global orient
    #traverse the path from initial postion to the desired provison through path
    if length != 0:
        send(initial,path[0])
        for i in range(0,length-1):
            send(path[i],path[i+1])
    else:
        send(initial,initial)
    if color == 'red1':
        #glow red led1
        ser.write("\x07")
    elif color == 'blue1':
        #glow blue led1
        ser.write("\x08")
    elif color == 'yellow1':
        #glow yellow led1
        ser.write("\x09")
    elif color == 'red2':
        #glow red led2
        ser.write("\x0A")
    elif color == 'blue2':
        #glow blue led2
        ser.write("\x0B")
    elif color == 'yellow2':
        #glow yellow led2
        ser.write("\x0C")
    time.sleep(1.5)
    return path[length-1]

'''
Function: drop()
input: initial position, path, path_length, colour of provision robot is going to deliver
output: end position of robot
Logic: traverse the path and switch-off the desired led
'''
def drop(initial,path,length,color):
    global orient
    #traverse the path from initial postion to the desired table through path
    if length !=0:
        send(initial,path[0])
        for i in range(0,length-1):
            send(path[i],path[i+1])
    else:
        send(initial,initial)
    if orient != 'e':
        if path[length-1][1] == 3:
            if orient == 'n':
                ser.write("\x16")
            else:
                ser.write("\x02")
            time.sleep(2)
        orient_correct(orient,'e',path[length-1])
        ser.write("\x16")
        time.sleep(2)
    else:
        if path[length-1][1] == 3:
            orient_correct(orient,'n',path[length-1])
            ser.write("\x16")
            time.sleep(2)
            orient_correct(orient,'e',path[length-1])
        ser.write("\x16")
        time.sleep(2)
    if color == 'red1':
        #off red led1
        ser.write("\x0D")
    elif color == 'blue1':
        #off blue led1
        ser.write("\x0E")
    elif color == 'yellow1':
        #off yellow led1
        ser.write("\x0F")
    elif color == 'red2':
        #off red led2
        ser.write("\x10")
    elif color == 'blue2':
        #off blue led2
        ser.write("\x11")
    elif color == 'yellow2':
        #off yellow led2
        ser.write("\x12")
    time.sleep(1.5)
    ser.write("\x02")
    time.sleep(1.5)
    return path[length-1]

'''
Function: path_plan()
input: co-ordinates of obstacles, start and final position
output: path and path length
Logic: gives the shortest path on the basis of Astar algorithm
'''
def path_plan(obstacles,start,final):
    #plans the path from start to final location
    map = ArenaMap()
    solution = Astar()
    grid = map.grid_map(obstacles,start,final)
    solution.init_grid(grid)
    path,length = solution.search()
    return path,length
    
class Cell(object):
    def __init__(self,x,y,reachable):
        #setting some parameters for each cell
        self.reachable=reachable
        self.x=x
        self.y=y
        self.parent= None
        self.cost=0
        self.heuristic=0
        #net_cost=cost+heuristic
        self.net_cost=0

'''
Class: Astar()
Functions: init_grid(); cell(); cell_heuristic(); neighbour(); update_cell(); display_path(); search();
input: grid
output: returns the shortest path between initial and final position and its path length
Logic: it selects the cell with least cost and move on till it reaches other end and gives the shortest path
'''
class Astar(object):
    def __init__(self):
        #list of unchecked neighbour cells
        self.open = []
        #keeps cells with lowest total_cost at top
        heapq.heapify(self.open)
        #list of already checked cells
        self.closed= set()
        #list of neighbour cells 
        self.cells=[]
  
    def init_grid(self,grid):
        for i in range(rows):
            for j in range(columns):
                #detecting the obstacles
                if grid[i][j]==1:
                    reachable= False
                else:
                    reachable= True
                self.cells.append(Cell(i,j,reachable))
                #detecting the start and end
                if(grid[i][j]==2):
                    self.start=self.cell(i,j)
                if(grid[i][j]==3):
                    self.end=self.cell(i,j)

    def cell(self,x,y):
        #returns the location to identify each cell
        return self.cells[x*columns+y]

    def cell_heuristic(self,cell):
        #returns the heuristic for astar algo
        return abs(cell.x-self.end.x)+abs(cell.y-self.end.y)

    def neighbour(self,cell):
        cells=[]
        #returns a list of neigbours of a cell
        if cell.x<columns-1:
            cells.append(self.cell(cell.x+1,cell.y))
        if cell.x>0:
            cells.append(self.cell(cell.x-1,cell.y))
        if cell.y<rows-1:
            cells.append(self.cell(cell.x,cell.y+1))
        if cell.y>0:
            cells.append(self.cell(cell.x,cell.y-1))
        return cells

    def update_cell(self,adj,cell):
        #update the details about the selected neigbour cell
        adj.cost=cell.cost+1
        adj.heuristic=self.cell_heuristic(adj)
        adj.parent=cell
        adj.net_cost= adj.cost + adj.heuristic

                        
    def display_path(self):
        #list for storing the path
        route_path =[]
        #flag to determine length of path
        count=0
        cell=self.end
        while cell.parent is not None:
            #storing the parents in list from end to start
            route_path.append([(cell.y)+1,(cell.x)+1])
            cell=cell.parent
            count+=1
        return route_path,count

    def search(self):
        #pushing the first element in open queue
        heapq.heappush(self.open,(self.start.net_cost,self.start))
        while(len(self.open)):
            net_cost,cell=heapq.heappop(self.open)
            #adding the checked cell to closed list
            self.closed.add(cell)
            if cell is self.end:
                #store path and path legth
                route_path, route_length = self.display_path()
                route_path.reverse()
                break
            #getting the adjoint cells 
            neighbours=self.neighbour(cell)
            for path in neighbours:
                if path.reachable and path not in self.closed:   #if cell is not an obstacle and has not been already checked
                    if (path.net_cost,path) in self.open:
                        if path.cost > cell.cost + 1:    #selecting the cell with least cost
                            self.update_cell(path,cell)
                    else:
                        self.update_cell(path,cell)
                        heapq.heappush(self.open,(path.net_cost,path))
        return route_path, route_length

'''
Function: play()
input: image
output: none
Logic: masking of obstacles, provisions and markers, initial position of robot,orientation, frequency of provision is calculated here
       robot is made to pick or drop the provision according to the frequency of provision asked and by calculating the shortest path using Astar algorithm
'''
def play(img):
    global data3
    global orient
    map = ArenaMap()
    #obstacles
    img_center = img[:,(w/5):(4*w/5),:]
    obstacles_masked = map.mask(img_center,low_green,high_green)
    obstacles = map.find_contours(obstacles_masked,1)
    print "obstacles",obstacles
    #blue markers
    blue = map.mask(img, low_blue, high_blue)
    supply_b, table_b = map.find_contours(blue,0)
    print "blue supply",supply_b
    print "blue table", table_b
    #red markers
    red = map.mask(img,low_red,high_red)
    supply_r, table_r = map.find_contours(red,0)
    print "red supply",supply_r
    print "red table",table_r
    #yellow markers
    yellow = map.mask(img,low_yellow,high_yellow)
    supply_y, table_y = map.find_contours(yellow,0)
    print "yellow supply",supply_y
    print "yellow table",table_y
    #bot
    violet = map.mask(img_center,low_violet,high_violet)
    v = map.find_contours(violet,1)
    orange = map.mask(img_center,low_orange,high_orange)
    o = map.find_contours(orange,1)
    initial = v[0]
    #checking initial position with respect to grid
    if initial[1] < 120:
        data3 = (3,1)
    elif initial[1] < 360:
        data3 = (3,3)
    else:
        data3 = (3,4)
    #finding orientation of robot
    orient = orientation(v[0],o[0])
    #finding frequency of blue provision required on table side
    freqz_b = len(table_b)
    #finding frequency of red provision required on table side
    freqz_r = len(table_r)
    #finding frequency of yellow provision required on table side
    freqz_y = len(table_y)
    #calculating max frequency
    freqz_max = max(freqz_b,freqz_y,freqz_r)

    if freqz_max == 1:
        #each table corresponds to different provision
        #calculating path and length from inital position of robot to each provison on supply side
        path_isb,length_isb = path_plan(obstacles,initial,supply_b[0])
        path_isr,length_isr = path_plan(obstacles,initial,supply_r[0])
        path_isy,length_isy = path_plan(obstacles,initial,supply_y[0])

        #calculating path and length from each supply provision to other supply provision
        path_srsb,length_srsb = path_plan(obstacles,supply_r[0],supply_b[0])
        path_sbsr,length_sbsr = path_plan(obstacles,supply_b[0],supply_r[0])
        path_srsy,length_srsy = path_plan(obstacles,supply_r[0],supply_y[0])
        path_sysr,length_sysr = path_plan(obstacles,supply_y[0],supply_r[0])
        path_sysb,length_sysb = path_plan(obstacles,supply_y[0],supply_b[0])
        path_sbsy,length_sbsy = path_plan(obstacles,supply_b[0],supply_y[0])

        #calculating path and length from each provision at supply side to each provision on table side
        path_srtb,length_srtb = path_plan(obstacles,supply_r[0],table_b[0])
        path_srty,length_srty = path_plan(obstacles,supply_r[0],table_y[0])
        path_srtr,length_srtr = path_plan(obstacles,supply_r[0],table_r[0])

        path_sbtb,length_sbtb = path_plan(obstacles,supply_b[0],table_b[0])
        path_sbty,length_sbty = path_plan(obstacles,supply_b[0],table_y[0])
        path_sbtr,length_sbtr = path_plan(obstacles,supply_b[0],table_r[0])

        path_sytb,length_sytb = path_plan(obstacles,supply_y[0],table_b[0])
        path_syty,length_syty = path_plan(obstacles,supply_y[0],table_y[0])
        path_sytr,length_sytr = path_plan(obstacles,supply_y[0],table_r[0])

        #calculating path and length from each provision at table side to each provision on supply side
        path_trsb,length_trsb = path_plan(obstacles,table_r[0],supply_b[0])
        path_trsy,length_trsy = path_plan(obstacles,table_r[0],supply_y[0])
        path_trsr,length_trsr = path_plan(obstacles,table_r[0],supply_r[0])

        path_tbsb,length_tbsb = path_plan(obstacles,table_b[0],supply_b[0])
        path_tbsy,length_tbsy = path_plan(obstacles,table_b[0],supply_y[0])
        path_tbsr,length_tbsr = path_plan(obstacles,table_b[0],supply_r[0])

        path_tysb,length_tysb = path_plan(obstacles,table_y[0],supply_b[0])
        path_tysy,length_tysy = path_plan(obstacles,table_y[0],supply_y[0])
        path_tysr,length_tysr = path_plan(obstacles,table_y[0],supply_r[0])

        #calculating path and length from each table provision to other table provision
        path_trtb,length_trtb = path_plan(obstacles,table_r[0],table_b[0])
        path_tbtr,length_tbtr = path_plan(obstacles,table_b[0],table_r[0])
        path_trty,length_trty = path_plan(obstacles,table_r[0],table_y[0])
        path_tytr,length_tytr = path_plan(obstacles,table_y[0],table_r[0])
        path_tytb,length_tytb = path_plan(obstacles,table_y[0],table_b[0])
        path_tbty,length_tbty = path_plan(obstacles,table_b[0],table_y[0])

        #finding the nearest provision om supply side from initial position of robot    
        minlen=min(length_isb,length_isr,length_isy)

        if(minlen==length_isy):
            #yellow provision is nearest on supply side so pick it
            if data3 == (3,3):
                if orient == 'e' or orient == 'w':
                    if path_isy[0][1] < data3 [1]:
                        orient_correct(orient,'n',data3)
                    else:
                        orient_correct(orient,'s',data3)
                ser.write("\x17")
                time.sleep(2)
            end = pick_up(data3,path_isy,length_isy,'yellow1')
            if(length_sysb<=length_sysr):
                #blue is near->move to blue provision -> pick it
                end = pick_up(end,path_sysb,length_sysb,'blue2')
                x='blue'
            else:
                #red is near->move to red provision -> pick it
                end = pick_up(end,path_sysr,length_sysr,'red2')
                x='red'
            if(x=='blue'):
                if(length_sbtb<=length_sbty):
                    #blue table is nearer than yellow table -> deliver it and then yellow
                    end = drop(end,path_sbtb,length_sbtb,'blue2')
                    end = drop(end,path_tbty,length_tbty,'yellow1')

                    #pick red provision and deliver it
                    end = pick_up(end,path_tysr,length_tysr,'red1')
                    end = drop(end,path_srtr,length_srtr,'red1')
                else:
                    #yellow table is nearer than blue table -> deliver it and then blue
                    end = drop(end,path_sbty,length_sbty,'yellow1')
                    end = drop(end,path_tytb,length_tytb,'blue2')

                    #pick red provision and deliver it
                    end = pick_up(end,path_tbsr,length_tbsr,'red1')
                    end = drop(end,path_srtr,length_srtr,'red1')
            if(x=='red'):
                if(length_srtr<=length_srty):
                    #red table is nearer than yellow table -> deliver it and then yellow
                    end = drop(end,path_srtr,length_srtr,'red2')
                    end = drop(end,path_trty,length_trty,'yellow1')

                    #pick blue provision and deliver it
                    end = pick_up(end,path_tysb,length_tysb,'blue1')
                    end = drop(end,path_sbtb,length_sbtb,'blue1')
                else:
                    #yellow table is nearer than red table -> deliver it and then red
                    end = drop(end,path_srty,length_srty,'yellow1')
                    end = drop(end,path_tytr,length_tytr,'red2')

                    #pick blue provision and deliver it
                    end = pick_up(end,path_trsb,length_trsb,'blue1')
                    end = drop(end,path_sbtb,length_sbtb,'blue1')                    

        #in similar manner we can pick and deliver the provision on the basis of nearest provision available if all different provisions are asked at table side
        
        elif(minlen==length_isr):
            if data3 == (3,3):
                if orient == 'e' or orient == 'w':
                    if path_isr[0][1] < data3 [1]:
                        orient_correct(orient,'n',data3)
                    else:
                        orient_correct(orient,'s',data3)
                ser.write("\x17")
                time.sleep(2)
            end = pick_up(data3,path_isr,length_isr,'red1')
            if(length_srsb<=length_srsy):
                end = pick_up(end,path_srsb,length_srsb,'blue2')
                x='blue'
            else:
                end = pick_up(end,path_srsy,length_srsy,'yellow2')
                x='yellow'
            if(x=='blue'):
                if(length_sbtb<=length_sbtr):
                    end = drop(end,path_sbtb,length_sbtb,'blue2')
                    end = drop(end,path_tbtr,length_tbtr,'red1')

                    end = pick_up(end,path_trsy,length_trsy,'yellow1')
                    end = drop(end,path_syty,length_syty,'yellow1')
                else:
                    end = drop(end,path_sbtr,length_sbtr,'red1')
                    end = drop(end,path_trtb,length_trtb,'blue2')

                    end = pick_up(end,path_tbsy,length_tbsy,'yellow1')
                    end = drop(end,path_syty,length_syty,'yellow1')
            if(x=='yellow'):
                if(length_syty<=length_sytr):
                    end = drop(end,path_syty,length_syty,'yellow2')
                    end = drop(end,path_tytr,length_tytr,'red1')

                    end = pick_up(end,path_trsb,length_trsb,'blue1')
                    end = drop(end,path_sbtb,length_sbtb,'blue1')
                else:
                    end = drop(end,path_sytr,length_sytr,'red1')
                    end = drop(end,path_trty,length_trty,'yellow2')

                    end = pick_up(end,path_tysb,length_tysb,'blue1')
                    end = drop(end,path_sbtb,length_sbtb,'blue1')                 
        elif(minlen==length_isb):
            if data3 == (3,3):
                if orient == 'e' or orient == 'w':
                    if path_isb[0][1] < data3 [1]:
                        orient_correct(orient,'n',data3)
                    else:
                        orient_correct(orient,'s',data3)
                ser.write("\x17")
                time.sleep(2)
            end = pick_up(data3,path_isb,length_isb,'blue1')
            if(length_sbsy<=length_sbsr):
                end = pick_up(end,path_sbsy,length_sbsy,'yellow2')
                x='yellow'
            else:
                end = pick_up(end,path_sbsr,length_sbsr,'red2')
                x='red'
            if(x=='yellow'):
                if(length_syty<length_sytb):
                    end = drop(end,path_syty,length_syty,'yellow2')
                    end = drop(end,path_tytb,length_tytb,'blue1')

                    end = pick_up(end,path_tbsr,length_tbsr,'red1')
                    end = drop(end,path_srtr,length_srtr,'red1')
                else:
                    end = drop(end,path_sytb,length_sytb,'blue1')
                    end = drop(end,path_tbty,length_tbty,'yellow2')

                    end = pick_up(end,path_tysr,length_tysr,'red1')
                    end = drop(end,path_srtr,length_srtr,'red1')
            if(x=='red'):
                if(length_srtr<length_srtb):
                    end = drop(end,path_srtr,length_srtr,'red2')
                    end = drop(end,path_trtb,length_trtb,'blue1')

                    end = pick_up(end,path_tbsy,length_tbsy,'yellow1')
                    end = drop(end,path_syty,length_syty,'yellow1')
                else:
                    end = drop(end,path_srtb,length_srtb,'blue1')
                    end = drop(end,path_tbtr,length_tbtr,'red2')

                    end = pick_up(end,path_trsy,length_trsy,'yellow1')
                    end = drop(end,path_syty,length_syty,'yellow1')
        ser.write("\x06")
        #buzzer
    elif freqz_max == 2:
        if freqz_max == freqz_r:
            #two table asked for red provision and now check for the provision asked by the third table
            if freqz_b == 1:
                #blue is the other provision asked for
                #calculating path and length from initial position to suppy blue and red
                path_isb,length_isb = path_plan(obstacles,initial,supply_b[0])
                path_isr,length_isr = path_plan(obstacles,initial,supply_r[0])
                path_srtr0,length_srtr0 = path_plan(obstacles,supply_r[0],table_r[0])
                path_srtr1,length_srtr1 = path_plan(obstacles,supply_r[0],table_r[1])
                if(length_srtr0 >= length_srtr1):
                    #if length of path from supply red to table_red_0 >= length of path from supply red to table_red_1 then we have to deliver first to table_red_1
                    # then to table_red_0 then pick up supply blue and deliver it
                    path_trsb, length_trsb = path_plan(obstacles,table_r[0],supply_b[0])
                    path_srtr,length_srtr = path_plan(obstacles,supply_r[0],table_r[1])
                    path_trtr,length_trtr = path_plan(obstacles,table_r[1],table_r[0])
                else:
                    #else we have to deliver first to table_red_0
                    # then to table_red_1 then pick up supply blue and deliver it
                    path_trsb, length_trsb = path_plan(obstacles,table_r[1],supply_b[0])
                    path_srtr,length_srtr = path_plan(obstacles,supply_r[0],table_r[0])
                    path_trtr,length_trtr = path_plan(obstacles,table_r[0],table_r[1])

                path_tbsr, length_tbsr = path_plan(obstacles,table_b[0],supply_r[0])
                path_sbtb, length_sbtb = path_plan(obstacles,supply_b[0],table_b[0])

                if((length_isb+length_tbsr)>=(length_isr+length_trsb)):
                    #if( sum of length of initial pos of bot and length of path from table blue to supply red)>= (sum of path length from initial to supply red and path length from table red to supply blue)
                    #then we have to pick red provision twice and deliver it
                    #then we have to pick supply blue and deliver it
                    if data3 == (3,3):
                        if orient == 'e' or orient == 'w':
                            if path_isr[0][1] < data3[1]:
                                orient_correct(orient,'n',data3)
                            else:
                                orient_correct(orient,'s',data3)
                        ser.write("\x17")
                    time.sleep(2)
                    end = pick_up(data3,path_isr,length_isr,'red1')
                    pick_up(end,path_isr,0,'red2')

                    end = drop(end,path_srtr,length_srtr,'red2')
                    end = drop(end,path_trtr,length_trtr,'red1')

                    end = pick_up(end,path_trsb,length_trsb,'blue1')
                    end = drop(end,path_sbtb, length_sbtb,'blue1')
                else:
                    #else we have to pick blue provision and deliver it
                    #then we have to pick supply red twice and deliver it
                    if data3 == (3,3):
                        if orient == 'e' or orient == 'w':
                            if path_isb[0][1] < data3[1]:
                                orient_correct(orient,'n',data3)
                            else:
                                orient_correct(orient,'s',data3)
                        ser.write("\x17")
                    end = pick_up(data3,path_isb,length_isb,'blue1')
                    end = drop(end,path_sbtb, length_sbtb,'blue1')

                    end = pick_up(end,path_tbsr,length_tbsr,'red1')
                    pick_up(end,path_tbsr,0,'red2')

                    end = drop(end,path_srtr,length_srtr,'red2')
                    end = drop(end,path_trtr,length_trtr,'red1')

            #### in similar way we can pick and deliver the required provisions if a provision is asked for twice and other provision for once

                    
            elif freqz_y == 1:
                path_isy,length_isy = path_plan(obstacles,initial,supply_y[0])
                path_isr,length_isr = path_plan(obstacles,initial,supply_r[0])
                path_srtr0,length_srtr0 = path_plan(obstacles,supply_r[0],table_r[0])
                path_srtr1,length_srtr1 = path_plan(obstacles,supply_r[0],table_r[1])
                if(length_srtr0 >= length_srtr1):
                    path_trsy, length_trsy = path_plan(obstacles,table_r[0],supply_y[0])
                    path_srtr,length_srtr = path_plan(obstacles,supply_r[0],table_r[1])
                    path_trtr,length_trtr = path_plan(obstacles,table_r[1],table_r[0])
                else:
                    path_trsy, length_trsy = path_plan(obstacles,table_r[1],supply_y[0])
                    path_srtr,length_srtr = path_plan(obstacles,supply_r[0],table_r[0])
                    path_trtr,length_trtr = path_plan(obstacles,table_r[0],table_r[1])

                path_tysr, length_tysr = path_plan(obstacles,table_y[0],supply_r[0])
                path_syty, length_syty = path_plan(obstacles,supply_y[0],table_y[0])

                if((length_isy+length_tysr)>=(length_isr+length_trsy)):
                    if data3 == (3,3):
                        if orient == 'e' or orient == 'w':
                            if path_isr[0][1] < data3[1]:
                                orient_correct(orient,'n',data3)
                            else:
                                orient_correct(orient,'s',data3)
                        ser.write("\x17")
                    end = pick_up(data3,path_isr,length_isr,'red1')
                    pick_up(end,path_isr,0,'red2')

                    end = drop(end,path_srtr,length_srtr,'red2')
                    end = drop(end,path_trtr,length_trtr,'red1')

                    end = pick_up(end,path_trsy,length_trsy,'yellow1')
                    end = drop(end,path_syty, length_syty,'yellow1')
                else:
                    if data3 == (3,3):
                        if orient == 'e' or orient == 'w':
                            if path_isy[0][1] < data3[1]:
                                orient_correct(orient,'n',data3)
                            else:
                                orient_correct(orient,'s',data3)
                        ser.write("\x17")
                    end = pick_up(data3,path_isy,length_isy,'yellow1')
                    end = drop(end,path_syty, length_syty,'yellow1')

                    end = pick_up(end,path_tysr,length_tysr,'red1')
                    pick_up(end,path_tysr,0,'red2')

                    end = drop(end,path_srtr,length_srtr,'red2')
                    end = drop(end,path_trtr,length_trtr,'red1')

        if freqz_max == freqz_b:
            if freqz_r == 1:
                path_isr,length_isr = path_plan(obstacles,initial,supply_r[0])
                path_isb,length_isb = path_plan(obstacles,initial,supply_b[0])
                path_sbtb0,length_sbtb0 = path_plan(obstacles,supply_b[0],table_b[0])
                path_sbtb1,length_sbtb1 = path_plan(obstacles,supply_b[0],table_b[1])
                if(length_sbtb0 >= length_sbtb1):
                    path_tbsr, length_tbsr = path_plan(obstacles,table_b[0],supply_r[0])
                    path_sbtb,length_sbtb = path_plan(obstacles,supply_b[0],table_b[1])
                    path_tbtb,length_tbtb = path_plan(obstacles,table_b[1],table_b[0])
                else:
                    path_tbsr, length_tbsr = path_plan(obstacles,table_b[1],supply_r[0])
                    path_sbtb,length_sbtb = path_plan(obstacles,supply_b[0],table_b[0])
                    path_tbtb,length_tbtb = path_plan(obstacles,table_b[0],table_b[1])

                path_trsb, length_trsb = path_plan(obstacles,table_r[0],supply_b[0])
                path_srtr, length_srtr = path_plan(obstacles,supply_r[0],table_r[0])

                if((length_isr+length_trsb)>=(length_isb+length_tbsr)):
                    if data3 == (3,3):
                        if orient == 'e' or orient == 'w':
                            if path_isb[0][1] < data3[1]:
                                orient_correct(orient,'n',data3)
                            else:
                                orient_correct(orient,'s',data3)
                        ser.write("\x17")
                    end = pick_up(data3,path_isb,length_isb,'blue1')
                    pick_up(end,path_isb,0,'blue2')

                    end = drop(end,path_sbtb,length_sbtb,'blue2')
                    end = drop(end,path_tbtb,length_tbtb,'blue1')

                    end = pick_up(end,path_tbsr,length_tbsr,'red1')
                    end = drop(end,path_srtr, length_srtr,'red1')
                else:
                    if data3 == (3,3):
                        if orient == 'e' or orient == 'w':
                            if path_isr[0][1] < data3[1]:
                                orient_correct(orient,'n',data3)
                            else:
                                orient_correct(orient,'s',data3)
                        ser.write("\x17")
                    end = pick_up(data3,path_isr,length_isr,'red1')
                    end = drop(end,path_srtr, length_srtr,'red1')

                    end = pick_up(end,path_trsb,length_trsb,'blue1')
                    pick_up(end,path_trsb,0,'blue2')

                    end = drop(end,path_sbtb,length_sbtb,'blue2')
                    end = drop(end,path_tbtb,length_tbtb,'blue1')


            elif freqz_y == 1:
                path_isy,length_isy = path_plan(obstacles,initial,supply_y[0])
                path_isb,length_isb = path_plan(obstacles,initial,supply_b[0])
                path_sbtb0,length_sbtb0 = path_plan(obstacles,supply_b[0],table_b[0])
                path_sbtb1,length_sbtb1 = path_plan(obstacles,supply_b[0],table_b[1])
                if(length_sbtb0 >= length_sbtb1):
                    path_tbsy, length_tbsy = path_plan(obstacles,table_b[0],supply_y[0])
                    path_sbtb,length_sbtb = path_plan(obstacles,supply_b[0],table_b[1])
                    path_tbtb,length_tbtb = path_plan(obstacles,table_b[1],table_b[0])
                else:
                    path_tbsy, length_tbsy = path_plan(obstacles,table_b[1],supply_y[0])
                    path_sbtb,length_sbtb = path_plan(obstacles,supply_b[0],table_b[0])
                    path_tbtb,length_tbtb = path_plan(obstacles,table_b[0],table_b[1])

                path_tysb, length_tysb = path_plan(obstacles,table_y[0],supply_b[0])
                path_syty, length_syty = path_plan(obstacles,supply_y[0],table_y[0])

                if((length_isy+length_tysb)>=(length_isb+length_tbsy)):
                    if data3 == (3,3):
                        if orient == 'e' or orient == 'w':
                            if path_isb[0][1] < data3[1]:
                                orient_correct(orient,'n',data3)
                            else:
                                orient_correct(orient,'s',data3)
                        ser.write("\x17")
                    end = pick_up(data3,path_isb,length_isb,'blue1')
                    pick_up(end,path_isb,0,'blue2')

                    end = drop(end,path_sbtb,length_sbtb,'blue2')
                    end = drop(end,path_tbtb,length_tbtb,'blue1')

                    end = pick_up(end,path_tbsy,length_tbsy,'yellow1')
                    end = drop(end,path_syty, length_syty,'yellow1')
                else:
                    if data3 == (3,3):
                        if orient == 'e' or orient == 'w':
                            if path_isy[0][1] < data3[1]:
                                orient_correct(orient,'n',data3)
                            else:
                                orient_correct(orient,'s',data3)
                        ser.write("\x17")
                    end = pick_up(data3,path_isy,length_isy,'yellow1')
                    end = drop(end,path_syty, length_syty,'yellow1')

                    end = pick_up(end,path_tysb,length_tysb,'blue1')
                    pick_up(end,path_tysb,0,'blue2')

                    end = drop(end,path_sbtb,length_sbtb,'blue2')
                    end = drop(end,path_tbtb,length_tbtb,'blue1')

        if freqz_max == freqz_y:
            if freqz_b == 1:
                path_isb,length_isb = path_plan(obstacles,initial,supply_b[0])
                path_isy,length_isy = path_plan(obstacles,initial,supply_y[0])
                path_syty0,length_syty0 = path_plan(obstacles,supply_y[0],table_y[0])
                path_syty1,length_syty1 = path_plan(obstacles,supply_y[0],table_y[1])
                if(length_syty0 >= length_syty1):
                    path_tysb, length_tysb = path_plan(obstacles,table_y[0],supply_b[0])
                    path_syty,length_syty = path_plan(obstacles,supply_y[0],table_y[1])
                    path_tyty,length_tyty = path_plan(obstacles,table_y[1],table_y[0])
                else:
                    path_tysb, length_tysb = path_plan(obstacles,table_y[1],supply_b[0])
                    path_syty,length_syty = path_plan(obstacles,supply_y[0],table_y[0])
                    path_tyty,length_tyty = path_plan(obstacles,table_y[0],table_y[1])

                path_tbsy, length_tbsy = path_plan(obstacles,table_b[0],supply_y[0])
                path_sbtb, length_sbtb = path_plan(obstacles,supply_b[0],table_b[0])

                if((length_isb+length_tbsy)>=(length_isy+length_tysb)):
                    if data3 == (3,3):
                        if orient == 'e' or orient == 'w':
                            if path_isy[0][1] < data3[1]:
                                orient_correct(orient,'n',data3)
                            else:
                                orient_correct(orient,'s',data3)
                        ser.write("\x17")
                    end = pick_up(data3,path_isy,length_isy,'yellow1')
                    pick_up(end,path_isy,0,'yellow2')

                    end = drop(end,path_syty,length_syty,'yellow2')
                    end = drop(end,path_tyty,length_tyty,'yellow1')

                    end = pick_up(end,path_tysb,length_tysb,'blue1')
                    end = drop(end,path_sbtb, length_sbtb,'blue1')
                else:
                    if data3 == (3,3):
                        if orient == 'e' or orient == 'w':
                            if path_isb[0][1] < data3[1]:
                                orient_correct(orient,'n',data3)
                            else:
                                orient_correct(orient,'s',data3)
                        ser.write("\x17")
                    end = pick_up(data3,path_isb,length_isb,'blue1')
                    end = drop(end,path_sbtb, length_sbtb,'blue1')

                    end = pick_up(end,path_tbsy,length_tbsy,'yellow1')
                    pick_up(end,path_tbsy,0,'yellow2')

                    end = drop(end,path_syty,length_syty,'yellow2')
                    end = drop(end,path_tyty,length_tyty,'yellow1')

            elif freqz_r == 1:
                path_isy,length_isy = path_plan(obstacles,initial,supply_y[0])
                path_isr,length_isr = path_plan(obstacles,initial,supply_r[0])
                path_syty0,length_syty0 = path_plan(obstacles,supply_y[0],table_y[0])
                path_srty1,length_srty1 = path_plan(obstacles,supply_y[0],table_y[1])
                if(length_syty0 >= length_syty1):
                    path_tysr, length_tysr = path_plan(obstacles,table_y[0],supply_r[0])
                    path_syty,length_syty = path_plan(obstacles,supply_y[0],table_y[1])
                    path_tyty,length_tyty = path_plan(obstacles,table_y[1],table_y[0])
                else:
                    path_tysr, length_tysr = path_plan(obstacles,table_y[1],supply_r[0])
                    path_syty,length_syty = path_plan(obstacles,supply_y[0],table_y[0])
                    path_tyty,length_tyty = path_plan(obstacles,table_y[0],table_y[1])

                path_trsy, length_trsy = path_plan(obstacles,table_r[0],supply_y[0])
                path_srtr, length_srtr = path_plan(obstacles,supply_r[0],table_r[0])

                if((length_isr+length_trsy)>=(length_isy+length_tysr)):
                    if data3 == (3,3):
                        if orient == 'e' or orient == 'w':
                            if path_isy[0][1] < data3[1]:
                                orient_correct(orient,'n',data3)
                            else:
                                orient_correct(orient,'s',data3)
                        ser.write("\x17")
                    end = pick_up(data3,path_isy,length_isy,'yellow1')
                    pick_up(end,path_isy,0,'yellow2')

                    end = drop(end,path_syty,length_syty,'yellow2')
                    end = drop(end,path_tyty,length_tyty,'yellow1')

                    end = pick_up(end,path_tysr,length_tysr,'red1')
                    end = drop(end,path_srtr, length_srtr,'red1')
                else:
                    if data3 == (3,3):
                        if orient == 'e' or orient == 'w':
                            if path_isr[0][1] < data3[1]:
                                orient_correct(orient,'n',data3)
                            else:
                                orient_correct(orient,'s',data3)
                        ser.write("\x17")
                    end = pick_up(data3,path_isr,length_isr,'red1')
                    end = drop(end,path_srtr, length_srtr,'red1')

                    end = pick_up(end,path_trsy,length_trsy,'yellow1')
                    pick_up(end,path_trsy,0,'yellow2')

                    end = drop(end,path_syty,length_syty,'yellow2')
                    end = drop(end,path_tyty,length_tyty,'yellow1')

        ser.write("\x06")
        #buzzer
    elif freqz_max == 3:
        #if single provision is asked thrice
        if freqz_max == freqz_b:
            #blue is asked thrice
            path_b,length_b = path_plan(obstacles,initial,supply_b[0])
            end = pick_up(data3,path_b,length_b,'blue1')
            if data3 == (3,3):
                if orient == 'e' or orient == 'w':
                    if path_isb[0][1] < data3[1]:
                        orient_correct(orient,'n',data3)
                    else:
                        orient_correct(orient,'s',data3)
                ser.write("\x17")

            #pick blue provision twice and deliver it to the table with minimum path length
            #move back to blue provison -> pick it once and deliver it
            pick_up(end,path_b,0,'blue2')
            path_1,length_1 = path_plan(obstacles,supply_b[0],table_b[0])
            path_2,length_2 = path_plan(obstacles,supply_b[0],table_b[1])
            path_3,length_3 = path_plan(obstacles,supply_b[0],table_b[2])
            length_min = min(length_1,length_2,length_3)
            if length_min == length_1:
    
                end = drop(end,path_1,length_1,'blue2')
                path,length = path_plan(obstacles,table_b[0],table_b[1])
                end = drop(end,path,length,'blue1')
                path,length = path_plan(obstacles,table_b[1],supply_b[0])
                end = pick_up(end,path,length,'blue1')
                path,length = path_plan(obstacles,supply_b[0],table_b[2])
                drop(end,path,length,'blue1')
            elif length_min == length_2:
                end = drop(end,path_2,length_2,'blue2')
                path, length = path_plan(obstacles,table_b[1],table_b[0])
                end = drop(end,path,length,'blue1')
                path, length = path_plan(obstacles,table_b[0],supply_b[0])
                end = pick_up(end,path,length,'blue1')
                path, length = path_plan(obstacles,supply_b[0],table_b[2])
                drop(end,path,length,'blue1')
            elif length_min == length_3:
                end = drop(end,path_3,length_3,'blue2')
                path, length = path_plan(obstacles,table_b[2],table_b[1])
                end = drop(end,path,length,'blue1')
                path, length = path_plan(obstacles,table_b[1],supply_b[0])
                end = pick_up(end,path,length,'blue1')
                path, length = path_plan(obstacles,supply_b[0],table_b[0])
                drop(end,path,length,'blue1')

        ### in similar way we can deliver the provision if a single provision is asked thrice

        
        elif freqz_max == freqz_y:
            path_y,length_y = path_plan(obstacles,initial,supply_y[0])
            end = pick_up(initial,path_y,length_y,'yellow1')
            pick_up(end,path_y,0,'yellow2')
            path_1,length_1 = path_plan(obstacles,supply_y[0],table_y[0])
            path_2,length_2 = path_plan(obstacles,supply_y[0],table_y[1])
            path_3,length_3 = path_plan(obstacles,supply_y[0],table_y[2])
            length_min = min(length_1,length_2,length_3)
            if length_min == length_1:
                end = drop(end,path_1,length_1,'yellow2')
                path,length = path_plan(obstacles,table_y[0],table_y[1])
                end = drop(end,path,length,'yellow1')
                path,length = path_plan(obstacles,table_y[1],supply_y[0])
                end = pick_up(end,path,length,'yellow1')
                path,length = path_plan(obstacles,supply_y[0],table_y[2])
                drop(end,path,length,'yellow1')
            elif length_min == length_2:
                end = drop(end,path_2,length_2,'yellow2')
                path, length = path_plan(obstacles,table_y[1],table_y[0])
                end = drop(end,path,length,'yellow1')
                path, length = path_plan(obstacles,table_y[0],supply_y[0])
                end = pick_up(end,path,length,'yellow1')
                path, length = path_plan(obstacles,supply_y[0],table_y[2])
                drop(end,path,length,'yellow1')
            elif length_min == length_3:
                end = drop(end,path_3,length_3,'yellow2')
                path, length = path_plan(obstacles,table_y[2],table_y[1])
                end = drop(end,path,length,'yellow1')
                path, length = path_plan(obstacles,table_y[1],supply_y[0])
                end = pick_up(end,path,length,'yellow1')
                path, length = path_plan(obstacles,supply_y[0],table_y[0])
                drop(end,path,length,'yellow1')
        elif freqz_max == freqz_r:
            path_r,length_r = path_plan(obstacles,initial,supply_r[0])
            end = pick_up(initial,path_r,length_r,'red1')
            pick_up(end,path_r,0,'red2')
            path_1,length_1 = path_plan(obstacles,supply_r[0],table_r[0])
            path_2,length_2 = path_plan(obstacles,supply_r[0],table_r[1])
            path_3,length_3 = path_plan(obstacles,supply_r[0],table_r[2])
            length_min = min(length_1,length_2,length_3)
            if length_min == length_1:
                end = drop(end,path_1,length_1,'red2')
                path,length = path_plan(obstacles,table_r[0],table_r[1])
                end = drop(end,path,length,'red1')
                path,length = path_plan(obstacles,table_r[1],supply_r[0])
                end = pick_up(end,path,length,'red1')
                path,length = path_plan(obstacles,supply_r[0],table_r[2])
                drop(end,path,length,'red1')
            elif length_min == length_2:
                end = drop(end,path_2,length_2,'red2')
                path, length = path_plan(obstacles,table_r[1],table_r[0])
                end = drop(end,path,length,'red1')
                path, length = path_plan(obstacles,table_r[0],supply_r[0])
                end = pick_up(end,path,length,'red1')
                path, length = path_plan(obstacles,supply_r[0],table_r[2])
                drop(end,path,length,'red1')
            elif length_min == length_3:
                end = drop(end,path_3,length_3,'red2')
                path, length = path_plan(obstacles,table_r[2],table_r[1])
                end = drop(end,path,length,'red1')
                path, length = path_plan(obstacles,table_r[1],supply_r[0])
                end = pick_up(end,path,length,'red1')
                path, length = path_plan(obstacles,supply_r[0],table_r[0])
                drop(end,path,length,'red1')
        ser.write("\x06")
        #buzzer

if __name__ == "__main__":
    
    cap = cv2.VideoCapture(1)
    ret, src = cap.read()
    cv2.imshow('src', src)
    cv2.imwrite("input_image.jpg", src)

    ##getting the perspective image
    img_src= get_perspective_image(src)

    cv2.imwrite("output_image.jpg", img_src)

    cv2.imshow('dst', img_src)
    
    img = img_src
    img = cv2.medianBlur(img,5) 
    h, w, c=img.shape
    play(img)
    
cv2.waitKey(0)
cap.release()
cv2.destroyAllWindows()
