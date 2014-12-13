from OSC import OSCServer,OSCClient, OSCMessage
import types

server = OSCServer(("192.168.8.6", 6000))


def arm_motors(path, tags, args, source):
  print source
  print args
  return 0

def handle_error(self,request,client_address):
  pass


server.addMsgHandler( "/1/arming",arm_motors)


server.handle_error = types.MethodType(handle_error, server)

while True:
  server.handle_request()

server.close()