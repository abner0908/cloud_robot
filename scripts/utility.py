import socket

def get_hostname():
    hostname = socket.gethostname()
    return  hostname.split('-')[0] + "_" + hostname.split('-')[1]

def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]
