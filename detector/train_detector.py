import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

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

model.save('grocery_detector_v3.pth')

# from detecto import utils, visualize, core

# model = core.Model.load('grocery_detector.pth', \
# 		['ambrosia','apple','banana','bottle','cereal','coke',\
# 				'lipton','lysol','milk','nutella','orange','oreo','pepsi'])

# image = utils.read_image('labels/pepsi.png')

# labels, boxes, scores = model.predict(image)
# predictions = model.predict_top(image)


# print(labels[0])
