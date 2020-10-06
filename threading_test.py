import threading, time
from threading import Timer


from threading import Timer, Event 
 
def every_so_often(): 
    if not done.is_set(): 
        print('Do the thing you want to every so often') 
        Timer(5.0, every_so_often).start() 


def main():
       
    global done 
    done = Event() 
    Timer(5.0, every_so_often).start() 
    
    ###
    t0 = time.time()

    while True:
        time.sleep(0.5)
        ans = time.time() - t0
        if ans > 11:
            print(ans)
            break

    print('Ende')
    ###
    
    done.set() 


if __name__ == '__main__':
    main()

