# Small device that shows the nearest plane using the ADSB Exchange API
#
# Copyright (c) 2021 John Graham-Cumming

from PIL import Image, ImageDraw, ImageFont, ImageOps
import os
import csv
import requests
import math

import RPi.GPIO as GPIO
import time
import neopixel
import board

import subprocess

# Contains API_KEY, MY_LAT, MY_LONG and RADIUS

from planes_config import API_KEY, MY_LAT, MY_LONG, RADIUS

# Contains the north and position variables and is used to avoid
# calibration position is the current position of the stepper motor in
# the range 0 to revolution-1. north is the LED that points to north.

from planes_position import north, position

# FUNCTIONS TO READ THE BLUE PUSH BUTTON

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

BUTTON_PIN = 23
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def button_wait():
    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
        pass

# FUNCTIONS FOR THE CIRCULAR STRIP OF LEDS THAT INDICATE DIRECTION TO
# THE AIRCRAFT

LED_COUNT = 24
STRIP_PIN = board.D18
strip = neopixel.NeoPixel(STRIP_PIN, LED_COUNT)

# Default brightness used for each of the RGB components of the LEDs'
# colour

led_intensity = 128

# strip_clear turns off every LED on the strip, call strip.show()
# to update the strip after calling this
def strip_clear():
    for i in range(0, LED_COUNT):
        strip[i] = (0, 0, 0)

# strip_spin lights up each LED on the strip in turn and finishes with
# them all off
def strip_spin():
    strip_clear()
    strip.show()

    for i in range(0, LED_COUNT):
        if i > 0:
            strip[i-1] = (0, 0, 0)
        strip[i] = (0, 0, led_intensity)
        strip.show()
        time.sleep(0.1)
        
    strip_clear()
    strip.show()

# calibrate_strip is used on start up to find the position of north
# where the device is installed. The user needs to hold the blue
# button down until the LED closest to north is illuminated. After 5
# seconds without touching the blue button the north position is fixed
# and returned by the function. This function leaves the LED pointing
# to north illuminated but in a different colour to show that the
# user's choice is confirmed
def calibrate_strip():
    strip_clear()
    strip.show()
    button_wait()

    i = 0
    strip[i] = (0, 0, led_intensity)
    strip.show()
    c = time.time()
    
    while (time.time() - c) < 5:
        if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
            strip[i] = (0, 0, 0)
            i = (i + 1) % LED_COUNT
            strip[i] = (0, 0, led_intensity)
            strip.show()
            time.sleep(0.1)
            c = time.time()

    strip[i] = (led_intensity, 0, 0)
    strip.show()
    return i
    
# FUNCTIONS TO CONTROL THE MODEL AIRCRAFT USED TO INDICATE THE TRACK
# OF THE AIRCRAFT

# These are the GPIO pins to which the four coils are connected

coil_A_pin = 4 
coil_B_pin = 17
coil_C_pin = 27
coil_D_pin = 22

# There are 'revolution' steps of the motor in a complete revolution
# and steps_per_degree steps per degree

revolution = 2038
steps_per_degree = revolution / 360.0

# This defines the sequence of coil activations for the stepper motor
# and current_step contains the step that was laste used to move the
# model aircraft

steps = 4
seq = list(range(steps))
seq[0] = [True, True, False,False]
seq[1] = [False,True, True, False]
seq[2] = [False,False,True, True]
seq[3] = [True, False,False,True]

current_step = 0

GPIO.setup(coil_A_pin, GPIO.OUT)
GPIO.setup(coil_B_pin, GPIO.OUT)
GPIO.setup(coil_C_pin, GPIO.OUT)
GPIO.setup(coil_D_pin, GPIO.OUT)

# motor_set_coils sets the coils on the stepper motor and is typically
# used with seq[] above
def motor_set_coils(a, b, c, d):
    GPIO.output(coil_A_pin, a)
    GPIO.output(coil_B_pin, b)
    GPIO.output(coil_C_pin, c)
    GPIO.output(coil_D_pin, d)

# motor_step moves the motor one step. The direction is determined by
# the clockwise parameter (True for clockwise) and this function
# updates position and current_step to keep track of the current motor
# position and which step in seq[] to use next
def motor_step(clockwise):
    global position
    global current_step

    if clockwise:
        current_step += 1
        position += 1
    else:
        current_step -= 1
        position -= 1

    current_step %= steps
    position %= revolution

    motor_set_coils(seq[current_step][0], seq[current_step][1],
                    seq[current_step][2], seq[current_step][3])

# motor_off turns off all the coils on the stepper motor. Since there
# is no torque on the motor needed between movements we can switch it
# off
def motor_off():
    motor_set_coils(False, False, False, False)

# plane_rotate moves the plane count steps in a clockwise or
# anti-clockwise direction with a delay of delay seconds between steps
def plane_rotate(delay, count, clockwise = True):
    for i in range(count):
        motor_step(clockwise)
        time.sleep(delay)
    motor_off()

# Since the stepper motor moves in units of 360/2038 degrees there will
# be errors in the position which accumulate over time. We keep track
# here and then fix the position when the error grows larger than a
# single step.
    
accumulated_error = 0.0
    
# plane_trak moves the plane to point to the angle trak degrees from
# north. It uses the position variable to determine the number of
# steps needed and goes by the shortest route (clockwise or
# anti-clockwise)
def plane_track(trak):
    d = trak * steps_per_degree - position
    delta = int(d)
    
    global accumulated_error
    accumulated_error += (d - delta)
    if abs(accumulated_error) >= 1:
        fix = int(accumulated_error)
        delta += fix
        accumulated_error -= fix
    
    clockwise = delta > 0
    delta = abs(delta)

    if delta > revolution/2:
        delta = revolution - delta
        clockwise = not clockwise
        
    plane_rotate(0.01, delta, clockwise)
    
# calibrate_plane is used to point the model aircraft to north on
# startup. The user rotates the the plane by holding down the blue
# button until it points in the right direction and then releases
# it. After five seconds with no pressure on the button the plane's
# position is set
def calibrate_plane():
    button_wait()

    c = time.time()
    
    while (time.time() - c) < 5:
        if GPIO.input(BUTTON_PIN) == GPIO.HIGH:
            plane_rotate(0.01, 4, True)
            c = time.time()

# findcsv reads a CSV file from filename and tries to find match in
# column col. If it finds it returns the row, if it doesn't it returns
# a fake row containing match. Yeah, this really should just read the
# CSV once on startup and make a dictionary but this allowed me to
# fiddle with the CSV files while the program was running
def findcsv(filename, col, match):
    with open(filename, 'r') as f:
        r = csv.reader(f)
        for row in r:
            if row[col] == match.strip():
                return row

    return [match, match, match, match, match]

# getplanes calls the ADBS Exchange API to get the JSON containing
# nearby planes. It returns the result of requests.get()
def getplanes():
    url = "https://adsbexchange-com1.p.rapidapi.com/json/lat/%.3f/lon/%.3f/dist/%d/" % (MY_LAT, MY_LONG, RADIUS)
    return requests.get(url,
      headers={
        "X-RapidAPI-Host": "adsbexchange-com1.p.rapidapi.com",
        "X-RapidAPI-Key": API_KEY,
        "Accept-Encoding": "None"
      })

# FUNCTIONS FOR DRAWING TEXT AND IMAGES ON THE SCREEN

# flag tries to find the flag of the country named in country
# by looking for a file called images/country.gif (any spaces
# in the country name are turned into -). If found it inserts
# the flag into img and then returns the new x position where
# its safe to write to the image and not overwrite the flag.
# All flags are resized to 38x25 for consistency
def flag(img, country, x, y):
    country_gif = 'images/' + country.lower() + '.gif'
    country_gif = country_gif.replace(' ', '-')
    
    if os.path.isfile(country_gif):
        country_img = Image.open(country_gif, 'r')
        img.paste(country_img.resize((38, 25)), (x, y+3))
        country_img.close()
        return x + 45

    return x

# The number of pixels to leave between lines of text on the screen

spacing = 4

last_text = ''

# text writes a line of text to d automatically adjusting the font
# size to fit the text on screen. It returns the new y position where
# text can be written based on the size of the text and the spacing
# value. Note that it uses last_text to automatically prevent the same
# string being written twice sequentially (this is done to eliminate
# airports that have the same name as the town they are in)
#
# The up parameter determines whether the text is being written top to
# bottom on the screen (up = False) or up from the bottom (up = True)
#
# The default (preferred) font size is s (in pt) and will
# automatically be reduced until the text fits across the screen
def text(d, x, y, t, s, up = False):
    global last_text
    if last_text == t:
        return y
    last_text = t
    
    while s >= 10:
        f = ImageFont.truetype('DejaVuSansMono.ttf', s)
        (w, h) = f.getsize(t)
        if w <= 320-x:
            if up:
                y -= h
                
            d.text((x, y), t, fill=(240, 240, 240), font=f)
            
            if up:
                return y - spacing
            else:
                return y + h + spacing
            
        s -= 2
    return y

# screen_backlight turns the backlight for the screen on or off
def screen_backlight(on):
    if on:
        v = 1
    else:
        v = 0

    subprocess.run('echo "%d" > /sys/class/backlight/soc:backlight/brightness' % v,
                   shell=True)

screen_tmp = '/tmp/planes.tmp.png'
screen_file = '/tmp/planes.png'
screen_links = ['/tmp/planes%d.png' % i for i in range(1, 4)]

# screen_show takes an image in img and writes it to a file and then
# uses fbi to draw it to the screen
def screen_show(img):
    
    # This is done to prevent fbi from getting an error if it tries to
    # read one of the images it is displaying while we write it. It's
    # written to a temporary file and then mv'ed into place.
    
    img.save(screen_tmp)
    subprocess.run('mv %s %s' % (screen_tmp, screen_file), shell=True)

    # Determine if there are any instance of fbi running. Start one if
    # there is not
    running = []
    try:
        running = subprocess.check_output(['pgrep', 'fbi']).decode("utf-8").strip().split('\n')
    except:
        pass

    if len(running) == 0:
        subprocess.run('fbi -t 1 -T 2 -a -cachemem 0 -noverbose -d /dev/fb1 %s' % ' '.join(screen_links),
                       shell=True)
    
# screen_start sets up the screen for use. The most important thing it
# does is create three symbolic links that are fed to fbi in
# screen_show. This is a trick to get fbi to cycle through images and
# allow a single fbi instance to updated smoothly
def screen_start():
    subprocess.run(['pkill', 'fbcp'])
    
    for l in screen_links:
        subprocess.run(['ln -s %s %s' % (screen_file, l)], shell=True)

# spotted is called when an aircraft has been found and it updates the
# screen, moves the model aircraft to track the actual aircraft and
# sets the LED strip to show where to look for it
def spotted(flight, airline, from_airport, from_city, from_country,
            to_airport, to_city, to_country, aircraft, altitude,
            bearing, trak):
    strip_clear()
    strip[(north+int(LED_COUNT*bearing/360)) % LED_COUNT] = (0,
                                                             led_intensity,
                                                             0)
    strip.show()

    img = Image.new('RGB', (320, 480), color = (0, 0, 0))
    d = ImageDraw.Draw(img)

    # Try to use a large font for the airline name
    
    top = 32
    if len(airline) > 15:
        top = 24

    y = 10    
    y = text(d, 10, y, airline, 32)
    y = text(d, 10, y, flight, 24)
    y += 20

    # TODO: do this on loading the CSV
    from_airport = from_airport.replace(' Airport', '')
    to_airport = to_airport.replace(' Airport', '')
    from_airport = from_airport.replace(' International', '')
    to_airport = to_airport.replace(' International', '')

    y = text(d, 10, y, from_airport, 24)
    y = text(d, 10, y, from_city, 24)
    from_country_offset = flag(img, from_country, 10, y)
    y = text(d, from_country_offset, y, from_country, 24)
    y += spacing * 2
    
    icon = Image.open('images/down.png', 'r')
    img.paste(icon, (10, y), icon)
    (w, h) = icon.size
    icon.close()
    y += h + spacing
    
    y = text(d, 10, y, to_airport, 24)
    y = text(d, 10, y, to_city, 24)
    to_country_offset = flag(img, to_country, 10, y)
    y = text(d, to_country_offset, y, to_country, 24)

    y = 480 - spacing
    y = text(d, 10, y, altitude + ' ft', 24, True)
    y = text(d, 10, y, aircraft, 24, True)
    y = text(d, 10, y, str(round(trak)) + '°', 24, True)

    img = ImageOps.flip(img)
    img = ImageOps.mirror(img)
    screen_show(img)
    screen_backlight(True)
    plane_track(trak)
    save_position()

# save_position saves the current plane position and calibrated north
# in planes_position.py so that when the program reloads it can avoid
# calibration
def save_position():
    f = open('planes_position.py', 'w')
    f.writelines(['north = %d\n' % north, 'position = %d\n' % position])
    f.close()
    
# haversine works out the distance on the Earth's surface between
# two points given a latitude and longitude. 
def haversine(la1, lo1, la2, lo2):
    phi1 = math.radians(la1)
    phi2 = math.radians(la2)
    delta_phi = math.radians(la2-la1)
    delta_lambda = math.radians(lo2-lo1)

    a = math.sin(delta_phi/2.0) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * \
        math.sin(delta_lambda/2.0) ** 2
    return 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

# distance returns the distance to an aircraft
def distance(a):
    return haversine(MY_LAT, MY_LONG, float(a['lat']), float(a['lon']))

# bearing works out the bearing of one lat/long from another
def bearing(la1, lo1, la2, lo2):
    lat1 = math.radians(la1)
    lat2 = math.radians(la2)

    diff = math.radians(lo2 - lo1)

    x = math.sin(diff) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
            * math.cos(lat2) * math.cos(diff))

    b = math.degrees(math.atan2(x, y))
    return (b + 360) % 360

# blank is used to ensure that the screen and LEDs are off when
# there's no activity. It shuts off the screen after drawing black
# image on it and shuts off the LEDs.
def blank():
    strip_clear()
    strip.show()
    img = Image.new('RGB', (320, 480), color = (0, 0, 0))
    d = ImageDraw.Draw(img)
    screen_show(img)
    screen_backlight(False)

# required contains a list of fields that must be present and
# non-empty in the returned JSON

required = ['call', 'type', 'opicao', 'from', 'to', 'lat', 'lon',
            'trak', 'gnd']

strip_spin()

if north == -1:
    north = calibrate_strip()
    calibrate_plane()
    position = 0
    save_position()
    
strip_clear()
strip.show()

screen_start()
blank()

# The default update_delay is 30 seconds. Until an aircraft is seen
# the code checks once every 30 seconds for new aircraft; once
# tracking a plane it updates every ten seconds. Once there are no
# more planes it goes back to checking every 30 seconds

no_planes_delay = 30
tracking_plane_delay = 10
update_delay = 0

while True:
    time.sleep(update_delay)
    
    planes = getplanes()
    j = planes.json()

    if j is None or j['ac'] is None:
        blank()
        continue

    # Build near so that it contains aircraft that have all the fields
    # in required and are not on the ground

    near = []
    for ac in j['ac']:
        ok = True
        for r in required:
            if r not in ac or ac[r].strip() == '':
                ok = False
                break
            
        if ok and ac['gnd'] == '0':
            near.append(ac)

    # If there are aircraft then sort them by distance from the device
    # and display the nearest
   
    if len(near) > 0:
        near.sort(key=distance)
        ac = near[0]
        flight = ac['call']
        plane = findcsv('planes.dat', 2, ac['type'])[0]
        airline = findcsv('airlines.dat', 4, ac['opicao'])[1]
        altitude = ac['alt']
        from_ = findcsv('airports.dat', 4, ac['from'][:4])
        from_airport = from_[1]
        from_city = from_[2]
        from_country = from_[3]
        to_ = findcsv('airports.dat', 4, ac['to'][:4])
        to_airport = to_[1]
        to_city = to_[2]
        to_country = to_[3]
        b = bearing(MY_LAT, MY_LONG, float(ac['lat']), float(ac['lon']))
        trak = float(ac['trak'])
        spotted(flight, airline, from_airport, from_city, from_country,
                to_airport, to_city, to_country, plane, altitude, b, trak)
        update_delay = tracking_plane_delay
    else:
        update_delay = no_planes_delay
        blank()

    
    
