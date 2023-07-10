import threading
import time
import logging
import multiprocessing

def daemon(function):
    def wrapper(*args, **kwargs):
        thread = threading.Thread(target=function, args=args, kwargs=kwargs, daemon=True)
        thread.start()
        return thread
    return wrapper

def timed(function):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        results = function(*args, **kwargs)
        end_time = time.time()
        logging.info(f"Execution time of function {function.__name__} was {end_time - start_time}.")
        return results
    return wrapper

def process(function):
    def wrapper(*args, **kwargs):
        p = multiprocessing.Process(target=function, args=args, kwargs=kwargs)
        p.start()
        return p
    return wrapper
