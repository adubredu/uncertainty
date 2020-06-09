from detecto import core, utils, visualize

dataset = core.Dataset('labels/')
model = core.Model(['ambrosia','apple','banana','bottle','cereal','coke',\
				'lipton','lysol','milk','nutella','orange','oreo','pepsi'])
model.fit(dataset)
print('Done training')
image = utils.read_image('labels/cereal.png')
predictions = model.predict(image)

labels,boxes,scores = predictions
print(labels)
print(scores)

model.save('grocery_detector.pth')