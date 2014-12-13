import smbus
import time
import sys
bus = smbus.SMBus(1)


print ("Please enter the address in hex.")
address = int(raw_input(), 16)
print("Working on Address: " +  str(address))


# def bearing255():
#         bear = bus.read_byte_data(address, 1)
#         return bear

# def bearing3599():
#         bear1 = bus.read_byte_data(address, 2)
#         bear2 = bus.read_byte_data(address, 3)
#         bear = (bear1 << 8) + bear2
#         bear = bear/10.0
#         return bear

def clear():
    """Clear screen, return cursor to top left"""
    sys.stdout.write('\033[2J')
    sys.stdout.write('\033[H')
    sys.stdout.flush()

while True:
	for x in xrange(1,50):
		data = bus.read_byte_data(address, x)
		sys.stdout.write("Loc: " + str(x) + " - D: " + str(data) + "\n") 
		sys.stdout.flush()

	time.sleep(0.5)	
	clear()

	# sys.stdout.flush()
    # print("Give location to read data for:")

    # loc = int(raw_input())

    # data = bus.read_byte_data(address, loc)
    # print data

    # time.sleep(1)

