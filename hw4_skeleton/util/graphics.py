import tkinter, sys, time, os
import math, random
from state import state
import matplotlib.pyplot as plt
white = (255,255,255)
black = (0,0,0)
maroon=(128,0,0)
dark_red=(139,0,0)
brown=(165,42,42)
firebrick=(178,34,34)
crimson=(220,20,60)
red=(255,0,0)
tomato=(255,99,71)
coral=(255,127,80)
dark_orange=(255,140,0)
orange=(255,165,0)
gold=(255,215,0)
dark_golden_rod=(184,134,11)
golden_rod=(218,165,32)
pale_golden_rod=(238,232,170)
dark_khaki=(189,183,107)
khaki=(240,230,140)
yellow=(255,255,0)
yellow_green=(154,205,50)
lawn_green=(124,252,0)
colors = [lawn_green,yellow,gold,orange,dark_orange,coral,tomato,red,crimson,firebrick,brown,dark_red,maroon]

class graphics:
    def __init__(self):
        self.line_distance = 150
        self._root_window2 = tkinter.Tk()
        self._root_window = self._root_window2
        self._canvas = None  # The canvas which holds graphics
        self._canvas_xs = None  # Size of canvas object
        self._canvas_ys = None
        self._canvas_x = None  # Current position on canvas
        self._canvas_y = None


    def formatColor(self, r, g, b):
        return '#%02x%02x%02x' % (int(r * 255), int(g * 255), int(b * 255))

    def formatColor2(self, r, g, b):
        return '#%02x%02x%02x' % (int(r), int(g), int(b))

    def text(self, x, y, contents, font='Helvetica', size=12, style='normal', anchor="nw", r = 0,g=0,b=0):
        font = (font, str(size), style)
        color = self.formatColor(r, g, b)
        return self._canvas.create_text(x, y, fill=color, text=contents, font=font, anchor=anchor)

    def polygon(self, coords, outlineColor, fillColor=None, filled=1, smoothed=1, behind=0, width=1):
        c = []
        for coord in coords:
            c.append(coord[0])
            c.append(coord[1])
        if fillColor == None: fillColor = outlineColor
        if filled == 0: fillColor = ""
        poly = self._canvas.create_polygon(c, outline=outlineColor, fill=fillColor, smooth=smoothed, width=width)
        if behind > 0:
            self._canvas.tag_lower(poly, behind)  # Higher should be more visible
        return poly

    def square(self, r, c, color, filled=1, behind=0):
        coords = [(r * self.line_distance, c * self.line_distance), ((r + 1) * self.line_distance, c * self.line_distance),
                  ((r + 1) * self.line_distance, (c + 1) * self.line_distance), (r * self.line_distance, (c + 1) * self.line_distance)]
        return self.polygon(coords, color, color, filled, 0, behind=behind)

    def draw_background(self):
        corners = [(0, 0), (0, self._canvas_ys), (self._canvas_xs, self._canvas_ys), (self._canvas_xs, 0)]
        self.polygon(corners, self._bg_color, fillColor = self._bg_color, filled = True, smoothed = False)

    def begin_graphics(self, width=640, height=480, title=None):
        color = self.formatColor(255, 255, 255)
        # Check for duplicate call
        if self._root_window is not None:
            self._root_window.destroy()

        # Save the canvas size parameters
        self._canvas_xs, self._canvas_ys = width - 1, height - 1
        self._canvas_x, self._canvas_y = 0, self._canvas_ys
        self._bg_color = color

        # Create the root window
        self._root_window = tkinter.Tk()
        #self._root_window.protocol('WM_DELETE_WINDOW', self._destroy_window)
        self._root_window.title(title or 'Graphics Window')
        self._root_window.resizable(0, 0)

        # Create the canvas object
        try:
            self._canvas = tkinter.Canvas(self._root_window, width=width, height=height)
            self._canvas.pack()
            self.draw_background()
        except:
            self._root_window = None
            raise

    def clear_screen(self, background=None):
        self._canvas.delete('all')
        self.draw_background()
        self._canvas_x, self._canvas_y = 0, self._canvas_ys

    def checkered(self, num_x, num_y, rowcol_prob=None):

        if rowcol_prob is not None:
            for r in range(num_x):
                for c in range(num_y):
                    if (r, c) in rowcol_prob.keys():
                        val = rowcol_prob[(r, c)]
                    else:
                        val = 0
                    color = colors[int(math.ceil(val * (len(colors) - 1)))]
                    self.square(r, c, self.formatColor2(color[0], color[1], color[2]))

        for r in range(num_x):
            for c in range(num_y):
                self.text(self.line_distance * (r), self.line_distance * (c), str(r) + ',' + str(c), style="bold", r=0, g=0, b=0, size=16)
        for x in range(self.line_distance, self.line_distance * (num_x + 1), self.line_distance):
            self._canvas.create_line(x, 0, x, self.line_distance * num_y, fill="#476042")
        for y in range(self.line_distance, self.line_distance * (num_y + 1), self.line_distance):
            self._canvas.create_line(0, y, self.line_distance * num_x, y, fill="#476042")

        if rowcol_prob is not None:
            c_ = (num_y + 1.2)
            w = num_x / len(colors)
            for col_ind in range(len(colors)):
                r_ = num_x * col_ind / len(colors)
                col = self.formatColor2(colors[col_ind][0], colors[col_ind][1], colors[col_ind][2])
                coords = [(r_ * self.line_distance, c_ * self.line_distance), ((r_ + w) * self.line_distance, c_ * self.line_distance),
                          ((r_ + w) * self.line_distance, (c_ + 0.5) * self.line_distance), (r_ * self.line_distance, (c_ + 0.5) * self.line_distance)]
                self.polygon(coords, col, col, 1, 0, 0)


            self.text(self.line_distance / 10, self.line_distance * (num_y + 1), "Low pickup probability", style="bold", r=0, g=0, b=0, size=16)
            self.text(self.line_distance * (num_x) - 1.5 * self.line_distance, self.line_distance * (num_y + 1), "High pickup probability", style="bold", r=0, g=0, b=0, size=16)

g = graphics()
num_x = 5
num_y = 4

def pickup_belief_color(pickup_distribution):
    max_prob = max(pickup_distribution.values())
    min_prob = min(pickup_distribution.values())
    rowcol_prob = {}
    orig_rowcol_prob = {}
    for (x,y) in pickup_distribution.keys():
        rowcol_prob[(x, y)] = round(float((pickup_distribution[(x,y)]- min_prob) / (max_prob - min_prob)), 6)
        orig_rowcol_prob[(x, y)] = round(float(pickup_distribution[(x,y)]), 6)
    return rowcol_prob, orig_rowcol_prob

def plot_distribution(dist, label):
    num_x = 5
    num_y = 4
    g = graphics()
    g.begin_graphics(width=(num_x + 2) * g.line_distance, height=(num_y + 2) * g.line_distance, title=label)

    rowcol_prob, orig_rowcol_prob = pickup_belief_color(dist)
    #print(orig_rowcol_prob)
    #print(rowcol_prob)
    g.checkered(num_x, num_y, rowcol_prob)
    for r in range(num_x):
        for c in range(num_y):
            photo_x = r * g.line_distance + g.line_distance / 3
            photo_y = c * g.line_distance + g.line_distance / 3
            if (r, c) not in orig_rowcol_prob.keys():
                orig_rowcol_prob[(r, c)] = 0
            #(num_y) * g.line_distance -
            g.text(photo_x - g.line_distance / 10, (photo_y - g.line_distance / 14), str(round(orig_rowcol_prob[(r, c)], 4)), style='bold', r=0, g=0,
                   b=0, size=int(24 * 4 / num_y))

    g._canvas.update()
    foldername = os.getcwd() + '/../data/' + label
    ps = g._canvas.postscript(colormode='color')
    img = Image.open(io.BytesIO(ps.encode('utf-8')))
    img.save(foldername + ".png", 'png')
    input("Enter any key to continue")
    #g.clear_screen()

def visualize_requests(all_new_requests):
    g.begin_graphics(width=(num_x + 2) * g.line_distance, height=(num_y + 2) * g.line_distance, title='requests')
    time = 0
    foldername = os.getcwd() + '/../data/trajectory_file/requests/'
    if not os.path.exists(foldername):
        os.mkdir(foldername)
    g.draw_background()
    g.checkered(num_x, num_y)
    for new_requests in all_new_requests:

        for request in new_requests:
            req_visual(request[0], request[1])

        g.square(1, num_y+0.1, g.formatColor2(white[0], white[1], white[2]))
        g.text(g.line_distance / 2, (num_y + 0.2) * g.line_distance, "Time: "+str(time), size = 36)
        g._canvas.update()



        ps = g._canvas.postscript(colormode='color')
        img = Image.open(io.BytesIO(ps.encode('utf-8')))
        img.save(foldername + "/frame" + str(time) + ".png", 'png')
        time += 1

    input("Enter any key to continue")
    os.system('ffmpeg -r 1 -start_number 0 -i ../data/trajectory_file/requests'
              + '/frame%0d.png -pix_fmt yuvj420p -vcodec mjpeg -f mov ../data/trajectory_file/requests.mov')


def plot_trajectory(trajectory, label1, label2, num_agents):
    #g = graphics()
    g.begin_graphics(width=(num_x + 2) * g.line_distance, height=(num_y + 2) * g.line_distance, title=label1)
    time = 0
    running_cost = 0
    for line in trajectory.split('\n'):
        if line == '':
            break
        #print(line)

        taxi_state_object = state(line, num_agents)
        g.draw_background()
        g.checkered(num_x, num_y)
        running_cost = visualize_taxis_requests(taxi_state_object, time, running_cost, label2)
        time += 1
        #input()
    os.system('ffmpeg -r 1 -start_number 0 -i ../data/trajectory_file/'+label2
              +'/frame%0d.png -pix_fmt yuvj420p -vcodec mjpeg -f mov ../data/trajectory_file/'+label2+'.mov')

from PIL import Image
import io
def req_visual(request_x, request_y):
    #g = graphics()
    photo = tkinter.PhotoImage(file= os.getcwd() +'/../util/request.png')
    photo = photo.subsample(int(2 * num_y), int(2 * num_y))
    label = tkinter.Label(image=photo)
    label.image = photo

    photo_x = request_x * g.line_distance + g.line_distance / 2 + g.line_distance * random.uniform(-1, 1) / 3
    photo_y = request_y * g.line_distance + g.line_distance / 2 + g.line_distance * random.uniform(-1, 1) / 3
    g._canvas.create_image((photo_x, photo_y), image=photo)

def taxi_visual(request_x, request_y, occupied = False):
    #g = graphics()
    if occupied:
        photo = tkinter.PhotoImage(file=os.getcwd() +'/../util/taxi_in_transit.png')
        photo = photo.subsample(int(6 * num_y / 4), int(6 * num_y / 4))
    else:
        photo = tkinter.PhotoImage(file=os.getcwd() +'/../util/taxi.png')
        photo = photo.subsample(int( 1.5*num_y ), int( 1.5*num_y ))
    label = tkinter.Label(image=photo)
    label.image = photo
    photo_x = request_x * g.line_distance + (g.line_distance / 2) + g.line_distance * random.uniform(-1, 1) / 3
    photo_y = request_y * g.line_distance + (g.line_distance / 2) + g.line_distance * random.uniform(-1, 1) / 3
    g._canvas.create_image((photo_x, photo_y), image=photo)

def visualize_taxis_requests(taxi_state_object, time, running_cost, label):
    for request in taxi_state_object.outstanding_requests:
        req_visual(request[0], request[1])
    for ell in range(len(taxi_state_object.agent_locations)):
        taxi_visual(taxi_state_object.agent_locations[ell][0], taxi_state_object.agent_locations[ell][1],
                    occupied=(taxi_state_object.time_left_in_current_trip[ell] > 0))

    running_cost += len(taxi_state_object.outstanding_requests)
    g.text(g.line_distance / 2, (num_y + 0.2) * g.line_distance, "Time: "+str(time)
           +"\tCost: "+str(running_cost), size = 36)
    g._canvas.update()
    #print('label',label)
    #exit()
    foldername = os.getcwd() + '/../data/trajectory_file/' + label+'/'
    if not os.path.exists(foldername):
        os.mkdir(foldername)
    ps = g._canvas.postscript(colormode='color')
    img = Image.open(io.BytesIO(ps.encode('utf-8')))
    img.save(foldername + "/frame" + str(time) + ".png", 'png')
    #plt.show()
    return running_cost