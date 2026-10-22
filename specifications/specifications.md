# Context

The OpenRCT2 project is an open-source implementation of the RCT2 game.
It lets the user build a golf course using an isometric view.

The goal of this project is to reuse the OpenRCT2 code to design an isometric, retro golf simulator game.

# Map

A tile is 5 yards long, so the map should be at least 256x256 to support 18 holes courses.
The map should have the following features:
- Different land textures, which affects the physics of the golf ball
- Different land elevations and land slopes
- Water
- Tiles with a water stream
- Buildings
- Pathways
- Benches
- Trash bins
- Many golfer peeps
- Animals
- Streets
- Cars
- Golf holes
- Driving range
- Mini putt course
- Placeable scenery
- 3d spatial sound sources

## Land textures and land slopes

The land textures should be the following:
- Sand
- Fairway
- Rough
- Deep rough
- Sand
- Woods
- Rocks

These land textures should affect the physics of the golf ball accordingly.
We should have the ability to define in 4 textures for a single tile, separated by their diagonals.
See tile_texture.png for an example: the 4 colours indicate different land textures.

In OpenRCT2, only one land slope was permitted. The game should let the player have 4 levels of slopes, one going up by a quarter height, one going up by a half step (the current one implement in OpenRCT2), one going up in 3/4 steps and one going 1 full step.

For this reason, there will be many, many more land sprites to have.
One way to reduce the amount of sprites is via sprite masking; for example, we store square texture for flat sand, fairway, rough and deep rough. When we want those 4 different textures on a single tile, we generate the tile texture procedurally through sprite masking since the texture limits on the tile are always the same.

That land texture instruction is also applied to the slopes.

When two different textures meet, there should also be a land texturing smoothing, which is present in OpenRCT2.

Those land textures should be loadable as objects, in the json format, similar to what OpenRCT2 does for its objects. It should have those fields:
- Roughness
- Ball bouncyness
- Name
- Description
- Cost to build
- Cost to remove
- Path to image in the zip file

This json file should be packed together with the sprites in a compressed file.
Each land texture would then be in their corresponding packed file, so that the game reads these at startup.

## Water

The water should be very similar to what OpenRCT2 has.

## Tiles with a water stream

The water stream can be on top of any regular land texture.
The water stream can go in the 4 directions.

A water stream is an another type of object, which has its json file with its sprites.

## Buildings

The buildings are obstacles for the golf balls, and the peeps can interact with these.
Here are some building types:

- Golf club : peeps go there to eat, drink, bladder, buy golf equipment
- Golf cottages : VIP peeps live there. The player should assign tiles where golf cottages can spawn. When the golf course get enough popularity, the player gets money from a golfer buying the land. When the land is bought, a golf cottage spawn.
- Food truck : peeps can go buy something to drink and eat

The buildings are also loadable objects, with a json file defining it and its sprites for the 4 views. It should also store information for its size and height, which will be used to calculate the collisions with the golf balls.

## Pathways

A pathway is half the size of a tile (2 meters). It should be used by peeps and peeps in golf carts. They are also loadable objects.

## Benches and trash bins

Benches and trash bins can be placed on pathways

## Golfers

The peeps in OpenRCT2 consists of 2d sprites; this poses a problem since it required many, many images for all the possible animations and views. For this game, peeps should be rendered in 3d, on top of the 2d isometric terrain. These peeps should have a rendered result similar to OpenRCT2.

These peeps can also drive cars to get access to the golf course and drive golf carts to go faster.
They carry some money when they spawn in the map. Two ways to spawn:
- On a spawn point, which spawn them in their cars 
- On a golf cottage if they own one. They spawn on foot.

The golf carts should be rendered at all time on the course, and the peeps must walk to these to drive them.
These golf carts are parked on specified tiles by the players; the golf peeps have to pay a fee to get access to one.
When they finish the course, they return the cart to any golf cart parking space.

The map should also have parking lots, which peeps can park their car and walk on foot.
If the parking spaces are full, no more cars can spawn on the map.

The cars and golfing carts are also rendered in 3d.
The terrain slopes should affect the rotation of the rendered models.

The golfers can have multiple actions:
- sitting on a bench
- talking to an another golfer
- striking a ball
- striking the ball with a mini putt club

The golfers, golf carts and carts are also defined in loadable objects, with their json and 3d file that contains their animations as well.

The golfers should have:
- Hunger
- Thristyness
- Energy
- Happyness

The golfers can go to different buildings for different needs. If the hole is too hard, the player's happyness should get lower.
At some point, if there is a problem with the course, a golfer can rage quit the park, insulting everyone on its path.

The golfers should also have different set of skills:
- Long play (how far they can hit the ball)
- Accuracy (how accurate they are)
- Foolproof (how resilient they are when they get tired)
- Physique (how much their stats drain down, someone in shape can play golf for a longer time)

Golfers can improve their skills at driving range, playing games, etc. They lose skill gradually when they don't play.

## Animals

There should also be animals on the course, rendered as 3d models as well.
These should also be loadable objects.

## Streets

Streets should let cars go around the map, and connect to a parking lot tile.
- One way roads
- Two way roads

They are very similar to the path objects in OpenRCT2.
They should also be loadable objects with their sprites.
Notice that the additional land slopes will change the number of sprites required.

## Golf Holes

A tile is 5 yards. So a 400 yards is 80 tiles.
The golf hole should have designated starting positions, up to 3, for man, woman and child.
The player start by placing the starting locations, from man to child, then place the hole.

The golf peeps always begin on the first hole; there should be a hole scheduling window, which the player can either let the game determine how many minutes it take to complete the hole, or manually set it.

The golf peeps will then arrive at the hole based on that schedule. When the golf peeps wait, their happiness get lower.

## Driving range

A driving range area can be delimited on a map. The player needs to define multiple tiles for letting the game know where the golfers shoot their balls. Some fishnets can be placed to block unwanted balls to go hit a golfer.

A driving range kiosk needs to be placed in the delimited area, preferably behind the golfers to avoid being hit.
The player can place distance signs, which their number is the distance from the kiosk to that sign, automatically generated by the game. The golfer would then know the distance it hits, increasing their happiness. The driving range increases the longplay skill.

## Mini putt course

Mini putt courses can also be placed, in the same way as regular courses but the only allowed club is the miniputt. The miniputt increase the accuracy skill. The regular courses increase all the skills. 

## Placeable scenery

This should be very similar to what OpenRCT2 does; the scenery are all placeable objects. The golf balls can collide with these, so they also need a height parameter. The golf ball, when it hits a scenery piece, bounces back.

## 3d spatial sound sources

The animals, golfers, cars, carts, etc. should emit a sound in 3d space. Their sounds are in their respective object packed files with their json.

# Goal of the game

The goal is to manage a golf course, and to have some profits.
Here are some features:

- Ability to have loans
- Ability to host tournaments: when a golf course meets some requirements, a tournament can be hosted. The player has to reserve some tiles for the requirement tournament infrastructure.


# Reuse of the RCT2 graphics

In all circumstances, we must AVOID being dependant on the original Rollercoaster Tycoon 2 game assets. This means no g2.dat, g1.dat, etc. This needs to be an open-source game. All assets should be in packed, compressed object files with their json.

No graphic should be reused from OpenRCT2 as well. When developping the code, we will use placeholders graphics, distinctive enough to know what is going on. No sound should also be reused.

