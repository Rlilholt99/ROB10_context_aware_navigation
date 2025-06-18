from owlready2 import *

# Create a new ontology
onto = get_ontology("http://example.org/robot.owl")

with onto:
    # === Classes ===
    class Room(Thing):
        pass

    class Object(Thing):
        pass

    class Need(Thing):
        pass

    # === Properties ===
    class located_in(Object >> Room):
        """Object is located in a Room"""
        pass

    class fulfills(Object >> Need):
        """Object fulfills a Need"""
        pass

# === Rooms ===
kitchen = onto.Room("Kitchen")
bedroom = onto.Room("Bedroom")
office  = onto.Room("Office")

# === Needs ===
thirsty    = onto.Need("Thirsty")
hungry     = onto.Need("Hungry")
rest       = onto.Need("Rest")
illuminate = onto.Need("Illuminate")
write      = onto.Need("Write")
compute    = onto.Need("Compute")

# === Objects common to multiple rooms ===
chair = onto.Object("Chair")
# Chairs can appear in kitchen, bedroom, office
chair.located_in.extend([kitchen, bedroom, office])
chair.fulfills.append(rest)  # e.g., sit and rest

# Generic table for multiple rooms
table = onto.Object("Table")
# Tables can be in kitchen and office
table.located_in.extend([kitchen, office])
# Tables support tasks but don't directly fulfill a primary need

# === Kitchen-specific Objects ===n
water_bottle = onto.Object("WaterBottle")
water_bottle.located_in.append(kitchen)
water_bottle.fulfills.append(thirsty)

sink = onto.Object("Sink")
sink.located_in.append(kitchen)

refrigerator = onto.Object("Refrigerator")
refrigerator.located_in.append(kitchen)

cabinet = onto.Object("Cabinet")
cabinet.located_in.append(kitchen)

countertop = onto.Object("Countertop")
countertop.located_in.append(kitchen)

cup = onto.Object("Cup")
cup.located_in.append(kitchen)
cup.fulfills.append(thirsty)

plate = onto.Object("Plate")
plate.located_in.append(kitchen)
plate.fulfills.append(hungry)

spoon = onto.Object("Spoon")
spoon.located_in.append(kitchen)
spoon.fulfills.append(hungry)

# New kitchen appliances
stove = onto.Object("Stove")
stove.located_in.append(kitchen)
stove.fulfills.append(hungry)

fridge = onto.Object("Fridge")
fridge.located_in.append(kitchen)
fridge.fulfills.append(hungry)

oven = onto.Object("Oven")
oven.located_in.append(kitchen)
oven.fulfills.append(hungry)

# === Bedroom-specific Objects ===n
bed = onto.Object("Bed")
bed.located_in.append(bedroom)
bed.fulfills.append(rest)

pillow = onto.Object("Pillow")
pillow.located_in.append(bedroom)
pillow.fulfills.append(rest)

blanket = onto.Object("Blanket")
blanket.located_in.append(bedroom)
blanket.fulfills.append(rest)

lamp = onto.Object("Lamp")
lamp.located_in.append(bedroom)
lamp.fulfills.append(illuminate)

cushion = onto.Object("Cushion")
cushion.located_in.append(bedroom)

sofa = onto.Object("Sofa")
sofa.located_in.append(bedroom)

# === Office-specific Objects ===
desk = onto.Object("Desk")
desk.located_in.append(office)

screen = onto.Object("Screen")
screen.located_in.append(office)

book = onto.Object("Book")
book.located_in.append(office)

office_chair = onto.Object("OfficeChair")
office_chair.located_in.append(office)
office_chair.fulfills.append(rest)

swiwel_chair = onto.Object("SwivelChair")
swiwel_chair.located_in.append(office)

computer = onto.Object("Computer")
computer.located_in.append(office)
computer.fulfills.append(compute)

pen = onto.Object("Pen")
pen.located_in.append(office)
pen.fulfills.append(write)

notebook = onto.Object("Notebook")
notebook.located_in.append(office)
notebook.fulfills.append(write)

crt_screen = onto.Object("CRTScreen")
crt_screen.located_in.append(office)

# === Save the ontology ===
onto.save(file="robot_ontology.owl", format="rdfxml")
