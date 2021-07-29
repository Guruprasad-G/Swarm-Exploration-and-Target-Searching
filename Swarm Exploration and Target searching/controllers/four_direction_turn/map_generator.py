from tkinter import *
from turtle import *
import turtle
from PIL import ImageGrab
def draw_wall(t):
    t.right(90)
    t.forward(25)
    t.penup()
    t.back(50)
    t.pendown()
    t.forward(25)


def getter(widget):
    root = widget.winfo_toplevel()
    widget.update()
    x=root.winfo_rootx()+widget.winfo_x()
    y=root.winfo_rooty()+widget.winfo_y()
    x1=x+widget.winfo_width()
    y1=y+widget.winfo_height()
    ImageGrab.grab().crop((x,y,x1,y1)).save("map.jpg")

def read_from_txt_file():
    map_file = open("Map_details.txt",'r')
    global_dict = {}
    for line in map_file:
        line_list = line.split(",")
        (r,u,l,d) = (line_list[4]=="True",line_list[5]=="True",line_list[6]=="True",line_list[7]=="True\n")
        global_dict[(float(line_list[1]),float(line_list[2]))] = global_dict.get((float(line_list[1]),float(line_list[2])),[r,u,l,d])
    print(global_dict)
    return global_dict

def generate_map(global_dict = {}):
    t = turtle.Turtle()
    t.hideturtle()
    if global_dict == {}:
        global_dict = read_from_txt_file()
    turtle.title("MAP")
    t.getscreen()
    t.speed(0)
    t.penup()
    for key,value in global_dict.items():
        t.goto(key[0]*50,key[1]*-50)
        t.pendown()
        t.penup()
        if not value[0]:
            t.setheading(0)
            t.forward(25)
            t.pendown()
            draw_wall(t)
            t.setheading(0)
        t.penup()
        t.goto(key[0]*50,key[1]*-50)
        if not value[1]:
            t.setheading(90)
            t.forward(25)
            t.pendown()
            draw_wall(t)
            t.setheading(90)
        t.penup()
        t.goto(key[0]*50,key[1]*-50)
        if not value[2]:
            t.setheading(180)
            t.forward(25)
            t.pendown()
            draw_wall(t)
            t.setheading(180)
        t.penup()
        t.goto(key[0]*50,key[1]*-50)
        if not value[3]:
            t.setheading(270)
            t.forward(25)
            t.pendown()
            draw_wall(t)
            t.setheading(270)
        t.penup()
        t.goto(key[0]*50,key[1]*50)
    keys = global_dict.keys()
    target_pos = list(keys)[-1]
    print("Target Pos =",target_pos)
    t.goto(target_pos[0]*50,target_pos[1]*50)
    t.dot(5)
    ts = t.getscreen()
    getter(ts.getcanvas())

if __name__ == '__main__':
    generate_map()
