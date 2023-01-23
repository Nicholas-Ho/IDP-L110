import sched, time
import urllib.request

url = 'http://192.168.4.1/' # Default Arduino AP IP
command = 'L'

def get(url, command, scheduler): 
    if command == 'H':
        command = 'L'
    else:
        command = 'H'

    # schedule the next call first
    scheduler.enter(1, 1, get, (url, command, scheduler,))
    url_led = url + command
    
    _ = urllib.request.urlopen(url_led) # <- How to send commands to the Arduino via Python

scheduler = sched.scheduler(time.time, time.sleep)
scheduler.enter(1, 1, get, (url, command, scheduler,))
scheduler.run()