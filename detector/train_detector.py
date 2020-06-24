import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

<<<<<<< HEAD
from detecto import core, utils, visualize
from torchvision import transforms

augmentations = transforms.Compose([
    transforms.ToPILImage(),
    transforms.RandomHorizontalFlip(0.5),
    transforms.ColorJitter(saturation=0.5),
    transforms.ToTensor(),
    utils.normalize_transform(),
])


dataset = core.Dataset('labels/',transform=augmentations)
model = core.Model(['baseball',
					  'beer',
					  'can_coke',
					  'can_pepsi',
					  'can_fanta',
					  'can_sprite',
					  'chips_can',
					  'coffee_box',
					  'cracker',
					  'cup',
					  'donut',
					  'fork',
					  'gelatin',
					  'meat',
					  'mustard',
					  'newspaper',
					  'orange',
					  'pear',
					  'plate',
					  'soccer_ball',
					  'soup',
					  'sponge',
					  'sugar',
					  'toy'])
model.fit(dataset)
print('Done training')
# image = utils.read_image('labels/bottle.png')
# predictions = model.predict(image)

# labels,boxes,scores = predictions
# print(labels)
# print(scores)

model.save('grocery_detector_v6.pth')

# from detecto import utils, visualize, core

# model = core.Model.load('grocery_detector_v6.pth', \
# 		['baseball',
# 		  'beer',
# 		  'can_coke',
# 		  'can_pepsi',
# 		  'can_fanta',
# 		  'can_sprite',
# 		  'chips_can',
# 		  'coffee_box',
# 		  'cracker',
# 		  'cup',
# 		  'donut',
# 		  'fork',
# 		  'gelatin',
# 		  'meat',
# 		  'mustard',
# 		  'newspaper',
# 		  'orange',
# 		  'pear',
# 		  'plate',
# 		  'soccer_ball',
# 		  'soup',
# 		  'sponge',
# 		  'sugar',
# 		  'toy'])

# image = utils.read_image('labels/1-7.png')

# labels, boxes, scores = model.predict(image)
# predictions = model.predict_top(image)


# print(labels)
# print(scores)
=======
# from detecto import core, utils, visualize
# from torchvision import transforms

# augmentations = transforms.Compose([
#     transforms.ToPILImage(),
#     transforms.RandomHorizontalFlip(0.5),
#     transforms.ColorJitter(saturation=0.5),
#     transforms.ToTensor(),
#     utils.normalize_transform(),
# ])


# dataset = core.Dataset('labels/',transform=augmentations)
# model = core.Model(['baseball',
# 					  'beer',
# 					  'can_coke',
# 					  'can_pepsi',
# 					  'can_fanta',
# 					  'can_sprite',
# 					  'chips_can',
# 					  'coffee_box',
# 					  'cracker',
# 					  'cup',
# 					  'donut',
# 					  'fork',
# 					  'gelatin',
# 					  'meat',
# 					  'mustard',
# 					  'newspaper',
# 					  'orange',
# 					  'pear',
# 					  'plate',
# 					  'soccer_ball',
# 					  'soup',
# 					  'sponge',
# 					  'sugar',
# 					  'toy'])
# model.fit(dataset)
# print('Done training')
# # image = utils.read_image('labels/bottle.png')
# # predictions = model.predict(image)

# # labels,boxes,scores = predictions
# # print(labels)
# # print(scores)

# model.save('grocery_detector_v6.pth')

from detecto import utils, visualize, core

model = core.Model.load('grocery_detector_v6.pth', \
		['baseball',
		  'beer',
		  'can_coke',
		  'can_pepsi',
		  'can_fanta',
		  'can_sprite',
		  'chips_can',
		  'coffee_box',
		  'cracker',
		  'cup',
		  'donut',
		  'fork',
		  'gelatin',
		  'meat',
		  'mustard',
		  'newspaper',
		  'orange',
		  'pear',
		  'plate',
		  'soccer_ball',
		  'soup',
		  'sponge',
		  'sugar',
		  'toy'])

image = utils.read_image('labels/1-7.png')

labels, boxes, scores = model.predict(image)
predictions = model.predict_top(image)


print(labels)
print(scores)
>>>>>>> 7a083f14c92c0a070ae27a81c752239e2791490c
