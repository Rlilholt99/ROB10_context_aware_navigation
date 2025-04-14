from owlready2 import *
# Create a new ontology
onto = get_ontology("http://example.org/robot.owl")

with onto:
    class Room(Thing):
        pass

    class Object(Thing):
        pass

    class located_in(Object >> Room):
        pass

    class KitchenObject(Object):
        pass
    class LivingRoomObject(Object):
        pass
    class BedroomObject(Object):
        pass
    

# Add sample data
kitchen = Room("Kitchen")
bottle = Object("WaterBottle")
bottle.located_in.append(kitchen)

chair = Object("Chair")
living_room = Room("LivingRoom")
chair.located_in.append(living_room)
chair.located_in.append(kitchen)

# Save the ontology
onto.save(file="robot_ontology.owl", format="rdfxml")
print("Ontology saved!")
