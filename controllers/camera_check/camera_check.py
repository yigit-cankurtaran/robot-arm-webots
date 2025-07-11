from controller import Supervisor

bot = Supervisor()
cam  = bot.getDevice('camera')
cam_node   = bot.getFromDevice(cam)
parent_def = cam_node.getParent().getDef()
print('camera parent =', parent_def)
