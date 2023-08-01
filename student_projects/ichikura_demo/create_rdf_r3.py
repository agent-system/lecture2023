#!/usr/bin/env python
# coding: utf-8

# In[1]:


import IPython.display
import sys
import json
from rdflib import *
from rdflib.collection import Collection
import glob
import os
import re


# In[2]:


def get_activity_program(activity_program_path):
    activity_program = []
    input_file = open(activity_program_path, "r")
    for line in input_file:
        activity_program.append(line.strip())
    input_file.close()
    return activity_program


# In[3]:


def get_graph_state_list(graph_state_path):
    graph_state_list = []
    for file_path in sorted(glob.glob(graph_state_path)):
        with open(file_path) as f:
            json_input = json.load(f)
            graph_state_list.append(json_input)
    return graph_state_list


# In[4]:


def getObjectName(line):
    #stepの行からすべてのobjectを抽出し、リストで返す
    result = []
    m = re.search(r'<.+>', line)
    if m is None:
        pass
    else:
        class_name = m.group()
        if "(" in class_name:
            #複数のobject
            class_name_list = re.split(r'\(\d+\)',class_name)
            for cn in class_name_list:
                cn = cn.replace("<","")
                cn = cn.replace(">","")
                cn = cn.strip()
                result.append(cn)
        else:
            #単一のobject
            class_name = class_name.replace("<","")
            class_name = class_name.replace(">","")
            class_name = class_name.strip()
            result.append(class_name)
        return result


# In[5]:


def getActionName(line):
    m = re.search(r'\[.+\]', line)
    if m is not None:
        action = m.group()
        action = action.replace("[","")
        action = action.replace("]","")
        action = action.strip()
        return action
    else:
        assert "there is no action"


# In[6]:


def getObjectId(line):
    result = []
    m = re.search(r'\(.+\)', line)
    if m is not None:
        id = m.group()
        if "(" in id:
            #複数のobject
            id_list = re.split(r'<.+>', id)
            for d in id_list:
                d = d.replace("(","")
                d = d.replace(")","")
                d = d.strip()
                result.append(d)
        else:
            result.append(id)
    else:
        assert "there is no ID"
    return result


# In[7]:


def getCharNode(nodes):
    result = None
    for node in nodes:
        if node['class_name'] == 'character':
            result = node
            break
    return result


# In[8]:


def createObjectState(g, node, state_cnt, activity_name):
    base = Namespace("http://example.org/virtualhome2kg/instance/")
    onto = Namespace("http://example.org/virtualhome2kg/ontology/")
    x3do = Namespace("https://www.web3d.org/specifications/X3dOntology4.0#")
#     affordance_instances = ["CAN_OPEN", "CUTTABLE", "DRINKABLE", "EATABLE", "GRABBABLE", "HANGABLE", "LIEABLE", "LOOKABLE", "MOVABLE", "POURABLE", "READABLE", "SITTABLE"]
#     object_property_instances = ["CREAM", "HAS_PAPER", "HAS_PLUG", "HAS_SWITCH", "RECIPIENT", "SURFACE"]
    attribute_map = {"CREAM": "cream", "HAS_PAPER": "has_paper", "HAS_PLUG": "has_plug", "HAS_SWITCH": "has_switch", "SURFACE": "has_surface", "CLOTHES": "clothes", "CONTAINERS":"containers", "COVER_OBJECT":"cover_object"}
    affordance_map = {"CAN_OPEN": "open", "CUTTABLE": "cut", "DRINKABLE": "drink", "EATABLE": "eat", "GRABBABLE": "grab", "HANGABLE": "hang", "LIEABLE": "lie", "LOOKABLE": "watch", "MOVABLE": "move", "POURABLE": "pour", "READABLE": "watch", "SITTABLE": "sit"}
    
    id = node['id']
    class_name = node['class_name']
    category = node['category']
    node_properties = node['properties']
    node_states = node['states']
    bounding_box = node['bounding_box']
    
    obj_state_r = base['state' + str(state_cnt) + '_' + class_name + str(id) + "_" + activity_name]
    g.add((obj_state_r, RDF.type, onto.State))
    
    g.add((obj_state_r, onto.isStateOf, base[class_name + str(id) + "_" + scene]))
    
    for vh_property in node_properties:
        if vh_property in attribute_map:
            #Attribute
            g.add((base[class_name + str(id) + "_" + scene], onto.attribute, onto[attribute_map[vh_property]]))

    for node_state in node_states:
#         #nodeが持つstateの値をStateクラスのインスタンスとして作成
#         if (base[node_state], None, None) not in g:
#             g.add((base[node_state], RDF.type, onto.StateType))
        g.add((obj_state_r, onto.state, onto[node_state]))

    #bounding_box
    shape = base['shape_state' + str(state_cnt) + '_' + class_name + str(id) + '_' + activity_name]
    g.add((shape, RDF.type, x3do.Shape))
    if bounding_box is not None:
#         shape = base['shape_state' + str(state_cnt) + '_' + class_name + str(id) + '_' + activity_name]
        bbox_center = BNode()
        bbox_size = BNode()
        c_list = []
        s_list = []
        for c in bounding_box['center']:
            c_list.append(Literal(c, datatype=XSD.double))

        for s in bounding_box['size']:
            s_list.append(Literal(s, datatype=XSD.double))

        g.add((bbox_center, RDF.type, x3do.SFVec3f))
        g.add((bbox_size, RDF.type, x3do.SFVec3f))
        Collection(g, bbox_center, c_list)
        Collection(g, bbox_size, s_list)

        g.add((shape, x3do.bboxCenter, bbox_center))
        g.add((shape, x3do.bboxSize, bbox_size))
    g.add((obj_state_r, onto.bbox, shape))
    
    return g, obj_state_r


# In[9]:


def getPreObjectState(g, state_cnt, class_name, id, activitiy_name):
    base = Namespace("http://example.org/virtualhome2kg/instance/")
    pre_obj_state_r = base['state' + str(state_cnt-1) + '_' + class_name + str(id) + "_" + activitiy_name]
    #前の状態があるか
    if (pre_obj_state_r, None, None) in g:
        #前の状態がある
        pass
    else:
        #前の状態がないということは、前の状態は「前の前」の状態（あるいはもっと前）と同じ
        pre_cnt=1
        while True:
            #前の状態が見つかるまで探す
            pre_cnt+=1
            #print([state_cnt, pre_cnt, class_name])
            pre_obj_state_r = base['state' + str(state_cnt-pre_cnt) + '_' + class_name + str(id) + "_" + activitiy_name]
            if (pre_obj_state_r, None, None) in g:
                break
    return pre_obj_state_r


# In[10]:


def createObjectAndSituation(g, graph_state_list, event_list, state_cnt, activity_name, scene):
    init_state_num = state_cnt
    base = Namespace("http://example.org/virtualhome2kg/instance/")
    onto = Namespace("http://example.org/virtualhome2kg/ontology/")
    ho = Namespace("http://www.owl-ontologies.com/VirtualHome.owl#")
    x3do = Namespace("https://www.web3d.org/specifications/X3dOntology4.0#")
    affordance_instances = ["CAN_OPEN", "CUTTABLE", "DRINKABLE", "EATABLE", "GRABBABLE", "HANGABLE", "LIEABLE", "LOOKABLE", "MOVABLE", "POURABLE", "READABLE", "SITTABLE"]
    object_property_instances = ["CREAM", "HAS_PAPER", "HAS_PLUG", "HAS_SWITCH", "PERSON", "RECIPIENT", "SURFACE"]
    for state in graph_state_list:
        nodes = state['nodes']
        edges = state['edges']
        home_situation_r = base["home_situation" + str(state_cnt) + "_" + activity_name]
        g.add((home_situation_r, RDF.type, onto.Situation))
        #nodes
        for node in nodes:
            id = node['id']
            class_name = node['class_name']
            node_properties = node['properties']
            node_states = node['states']
            
            obj_r = base[class_name + str(id) + "_" + scene]
            category_r = None
            
            if node['category'] != "Rooms": # Room is defined by vh2kg_ontology
                category_r = onto[node['category'].capitalize()]
                # category
                if (category_r, None, None) not in g:
                    g.add((category_r, RDF.type, OWL.Class))
                    g.add((category_r, RDFS.subClassOf, onto.Object))
                    g.add((category_r, RDFS.label, Literal(node['category'])))
            
            if (obj_r, None, None) not in g:
                object_class_r = onto[class_name.capitalize()]
                g.add((obj_r, RDF.type, object_class_r))
                g.add((obj_r, RDFS.label, Literal(class_name)))
                g.add((obj_r, DCTERMS.identifier, Literal(str(id))))
                #ObjectType
                if (object_class_r, None, None) not in g:
                    g.add((object_class_r, RDF.type, OWL.Class))
                    if category_r != None:
                        g.add((object_class_r, RDFS.subClassOf, category_r))
            
            if state_cnt == 0:
                #create object states
                g, obj_state_r = createObjectState(g, node, state_cnt, activity_name)
                g.add((obj_state_r, onto.partOf, home_situation_r))
            
            else:
                diff_flag = False
                pre_obj_state_r = getPreObjectState(g, state_cnt, class_name, id, activity_name)

                '''
                    compare between current and previous states
                '''
                #comparing affordance
                
                pre_obj_state_afford_list = [o.replace(onto,'') for s, p, o in g.triples((pre_obj_state_r,  onto.affordance, None))]
                for afford in pre_obj_state_afford_list:
                    if afford in affordance_instances:
                        if afford not in node_properties:
                            diff_flag = True
                            break
                
                #comparing affordance
                if diff_flag == False:
                    for afford in node_properties:
                        if afford in affordance_instances:
                            if afford not in pre_obj_state_afford_list:
                                diff_flag = True
                                break
                
                #comparing states
                if diff_flag == False:
                    pre_obj_state_state_list = [o.replace(onto,'') for s, p, o in g.triples((pre_obj_state_r,  onto.state, None))]
                    for pre_state in  pre_obj_state_state_list:
                        if pre_state not in node_states:
                            diff_flag = True
                            break
                            
                    if diff_flag == False:
                        for node_state in  node_states:
                            if node_state not in pre_obj_state_state_list:
                                diff_flag = True
                                break
                                
                #comparing spatial relations
                if diff_flag == False:
                    pre_graph_state = graph_state_list[state_cnt-1]
                    pre_nodes = pre_graph_state["nodes"]
                    pre_edges = pre_graph_state["edges"]
                    obj_pre_relations = [pre_edge for pre_edge in pre_edges if pre_edge["from_id"] == id]
                    obj_current_relations =  [edge for edge in edges if edge["from_id"] == id]
                    edge_change_flag = False
                    for obj_pre_relation in obj_pre_relations:
                        for obj_current_relation in obj_current_relations:
                            if obj_pre_relation == obj_current_relation:
                                break
                            else:
                                edge_change_flag = True
                        if edge_change_flag:
                            diff_flag = True
                            break
#                     if (pre_obj_state_r, onto.bbox, None) in g:
#                         pre_obj_state_shape =  [x for x in g.objects(pre_obj_state_r, onto.bbox)][0]
#                         pre_obj_state_location_bboxes =  [x for x in g.objects(pre_obj_state_shape, onto.inside)]
#                         pre_obj_state_location = None
#                         for pre_obj_state_location_bbox in pre_obj_state_location_bboxes:
#                             pre_obj_state_location_state = [x for x in g.subjecs(onto.bbox, pre_obj_state_location_bbox)][0]
#                             pre_obj_state_location = [x for x in g.objects(pre_obj_state_location_state, onto.isStateOf)][0]
#                             pre_obj_state_location_type = [x for x in g.objects(pre_obj_state_location, RDF.type)][0]
#                             if (pre_obj_state_location_type, RDFS.subClassOf, None) not in g: #if pre_obj_state_location_type is subClassOf :Room
#                                 break
                                
#                         pre_obj_state_location_id = [x for x in g.objects(pre_obj_state_location, DCTERMS.identifier)][0]
#                         for edge in edges:
#                             if edge["from_id"] == id:
#                                 if edge["relation_type"] == "INSIDE":
#                                     edge["to_id"]
#                                     if edge["to_id"] != pre_obj_state_location_id.value:
#                                         diff_flag = True
#                                     else:
                                        
                
                #bbox
                '''
                if diff_flag == False:
                    if (pre_obj_state_r, onto.bbox, None) in g:
                        pre_obj_state_shape =  [x for x in g.objects(pre_obj_state_r, onto.bbox)][0]
                        pre_obj_state_bboxCenter = [x for x in g.objects(pre_obj_state_shape, x3do.bboxCenter)][0]
                        pre_obj_state_x = [x for x in g.objects(pre_obj_state_bboxCenter, RDF.first)][0]
                        pre_obj_state_x_rest = [x for x in g.objects(pre_obj_state_bboxCenter, RDF.rest)][0]
                        pre_obj_state_y = [y for y in g.objects(pre_obj_state_x_rest, RDF.first)][0]
                        pre_obj_state_y_rest = [y for y in g.objects(pre_obj_state_x_rest, RDF.rest)][0]
                        pre_obj_state_z = [z for z in g.objects(pre_obj_state_y_rest, RDF.first)][0]
                        if node['bounding_box']['center'] != [pre_obj_state_x.value, pre_obj_state_y.value, pre_obj_state_z.value]:
                            diff_flag = True
                            print("state_cnt:" + str(state_cnt) + " " + class_name)
                '''
                        
                '''
                    前の状態との比較終了
                '''  
                
                if diff_flag == False:
                    #前の状態と同じ
                    g.add((pre_obj_state_r, onto.partOf, home_situation_r))
                else:
                    #前の状態と違う
                    g, obj_state_r = createObjectState(g, node, state_cnt, activity_name)
                    g.add((obj_state_r, onto.partOf, home_situation_r))
                    g.add((pre_obj_state_r, onto.nextState, obj_state_r))
                    g.add((obj_state_r, onto.previousState, pre_obj_state_r))
        
        #edges
        for edge in edges:
            from_id = edge["from_id"]
            to_id = edge["to_id"]
            if from_id == to_id:
                continue
            relation_type = edge["relation_type"].lower()
            from_obj_r = [x for x in g.subjects(DCTERMS.identifier, Literal(str(from_id)))][0]
            from_class_name = [x for x in g.objects(from_obj_r, RDFS.label)][0]
            from_obj_state_r = base['state' + str(state_cnt) + '_' + from_class_name + str(from_id) + '_' + activity_name]
            #前の状態がない場合、更に前の状態を取得
            if (from_obj_state_r, None, None) not in g:
                from_obj_state_r = getPreObjectState(g, state_cnt, from_class_name, from_id, activity_name)
            
            #shapeを取得
            if (from_obj_state_r, onto.bbox, None) in g:
                from_shape_r = [x for x in g.objects(from_obj_state_r, onto.bbox)][0]
            else:
                from_shape_r = base['shape_state' + str(state_cnt) + '_' + from_class_name + str(from_id) + '_' + activity_name]
            
            to_obj_r = [x for x in g.subjects(DCTERMS.identifier, Literal(str(to_id)))][0]
            to_class_name = [x for x in g.objects(to_obj_r, RDFS.label)][0]
            to_obj_state_r = base['state' + str(state_cnt) + '_' + to_class_name + str(to_id) + '_' + activity_name]
           #前の状態がない場合、更に前の状態を取得
            if (to_obj_state_r, None, None) not in g:
                to_obj_state_r = getPreObjectState(g, state_cnt, to_class_name, to_id, activity_name)
            
            #shapeを取得
            if (to_obj_state_r, onto.bbox, None) in g:
                to_shape_r = [x for x in g.objects(to_obj_state_r, onto.bbox)][0]
            else:
                to_shape_r = base['shape_state' + str(state_cnt) + '_' + to_class_name + str(to_id) + '_' + activity_name]
                
            g.add((from_shape_r,  onto[relation_type], to_shape_r))
        
        state_cnt += 1
    
    i = 0
    for event_r in event_list:
        before_home_situation_r = base["home_situation" + str(init_state_num + i) + '_' + activity_name]
        after_home_situation_r = base["home_situation" + str(init_state_num + i+1) + '_' + activity_name]
        g.add((event_r, onto.situationBeforeEvent, before_home_situation_r))
        g.add((event_r, onto.situationAfterEvent, after_home_situation_r))
        g.add((before_home_situation_r, onto.nextSituation, after_home_situation_r))
        i += 1
    
    return g, state_cnt


# In[11]:


def getEventResourceList(g, list_of_steps, event_id, activity_name, char_r):
    action_name_dic = {
        'LOOKAT': 'LookAt',
        'PLUGIN': 'PlugIn',
        'PLUGOUT': 'PlugOut',
        'POINTAT': 'PointAt',
        'PUTBACK': 'PutBack',
        'STANDUP': 'StandUp',
        'TURNTO': 'TurnTo',
        'PUTOBJBACK': 'PutObjBack',
        'SWITCHOFF': 'SwitchOff',
        'SWITCHON': 'SwitchOn',
        'WAKEUP': 'WakeUp'
    }
    base = Namespace("http://example.org/virtualhome2kg/instance/")
    an = Namespace("http://example.org/virtualhome2kg/ontology/action/")
    onto = Namespace("http://example.org/virtualhome2kg/ontology/")
    ho = Namespace("http://www.owl-ontologies.com/VirtualHome.owl#")
    time = Namespace("http://www.w3.org/2006/time#")
    event_list = []
    for step in list_of_steps:
        step = step.replace("<char0>","").strip()
        object_list = getObjectName(step)
        action = getActionName(step)
        object_id_list = getObjectId(step)

        steptype = None
        if action in action_name_dic:
            print(action)
            steptype = action_name_dic[action]
        else:
            steptype = action.capitalize()

        action = action.lower()
        action_r = an[action]
       
        # eventリソースのトリプル
        event_r = base["event" + str(event_id) + "_" + activity_name]
        if event_id == 0:
            g.add((event_r, RDF.type, onto.StartEvent))
        elif event_id == (len(list_of_steps)-1):
            g.add((event_r, RDF.type, onto.EndEvent))
        else:
            g.add((event_r, RDF.type, onto.Event))
        g.add((event_r, onto.eventNumber, Literal(event_id, datatype=XSD.int)))
        g.add((event_r, onto.action, action_r))
        g.add((event_r, onto.agent, char_r))
        

        #duration_r = base["time_" + action + "_" + activity_name]
        #g.add((duration_r, RDF.type, time.Duration))
        #g.add((duration_r, time.numericDuration, Literal(duration, datatype=XSD.decimal)))
        #g.add((duration_r, time.unitType, time.unitSecond))
        #g.add((action_r, time.hasDuration, duration_r))
        
        ##初出action
        #if (base[steptype], None, None) not in g:
        #    g.add((base[steptype], RDF.type, OWL.Class))
        #    g.add((base[steptype], RDFS.subClassOf, base.Action))

        #add main/target object
        try:
            if len(object_list) == 1:
                g.add((event_r, onto.mainObject, base[object_list[0] + object_id_list[0] + "_" + scene]))
            elif len(object_list) == 2:
                g.add((event_r, onto.mainObject, base[object_list[0] + object_id_list[0] + "_" + scene]))
                g.add((event_r, onto.targetObject, base[object_list[1] + object_id_list[1] + "_" + scene]))
        except Exception as e:
            print(e.args)
        

        if len(event_list) > 0:
            g.add((event_list[event_id-1], onto.nextEvent, event_r))
            g.add((event_r, onto.previousEvent, event_list[event_id-1]))
        event_list.append(event_r)
        event_id += 1
    return g, event_list


# In[12]:


def create_rdf(graph_state_list, program_description, activity_program, scene, directory):
    base = Namespace("http://example.org/virtualhome2kg/instance/")
    onto = Namespace("http://example.org/virtualhome2kg/ontology/")
    an = Namespace("http://example.org/virtualhome2kg/ontology/action/")
    ho = Namespace("http://www.owl-ontologies.com/VirtualHome.owl#")
    x3do = Namespace("https://www.web3d.org/specifications/X3dOntology4.0#")
    time = Namespace("http://www.w3.org/2006/time#")
    g = Graph()
    g.bind("ex", base)
    g.bind("vh2kg", onto)
    g.bind("vh2kg-an", an)
    g.bind("ho", ho)
    g.bind("x3do", x3do)
    g.bind("owl", OWL)
    g.bind("time", time)
    
    init_state = graph_state_list[0]
    nodes = init_state["nodes"]
    edges = init_state["edges"]
    
    #character
    char_node = getCharNode(nodes)
    char_class_name = char_node['class_name']
    char_id = char_node['id']
    char_r = base[char_class_name + str(char_id) + "_" + scene]
    g.add((char_r, RDF.type, onto.Character))
    g.add((char_r, RDFS.label, Literal(char_class_name)))
    g.add((char_r, DCTERMS.identifier, Literal(str(char_node['id']))))
    
    #activity
    id = 0
    state_cnt = 0
    
    activity_name = program_description["name"].lower().replace(" ","_") + directory + "_" + scene
    activity_r = base[activity_name]
    g.add((activity_r, RDFS.label, Literal(program_description["name"])))
    g.add((activity_r, RDFS.comment, Literal(program_description["description"])))
    g.add((activity_r, RDF.type, ho[program_description["name"].lower().replace(" ","_")]))

    #event
    event_id = id
    g, event_list = getEventResourceList(g, activity_program, event_id, activity_name, char_r)

    for event_r in event_list:
        g.add((activity_r, onto.hasEvent, event_r))
    #event関係終了

    #create objects and its situations
    g, state_cnt = createObjectAndSituation(g, graph_state_list, event_list, state_cnt, activity_name, scene)
            
    
    #Activity
    #.add((char_r, onto.activity, activity_r))
    g.add((activity_r, onto.agent, char_r))
    scene_r = base[scene]
    g.add((scene_r, RDF.type, onto.VirtualHome))
    g.add((activity_r, onto.virtualHome, scene_r))
    
    output_path = "rdf/" + scene + "_20220602/virtualhome2kg-" + folder.replace(" ", "_") + "-" + directory +  ".ttl"
    g.serialize(destination=output_path, format="turtle")


# In[13]:


scene = 'scene1'
folders = os.listdir("graph_state_list/" + scene + "/")
for folder in folders:
    try:
        activity_directory = "graph_state_list/" + scene + "/" + folder
        directories = os.listdir(activity_directory)
        program_discription_list = []
        for directory in directories:    
            program_description_path = "graph_state_list/" + scene + "/" + folder + "/" + directory + "/program-description.txt"
            program_description = {}
            input_file = open(program_description_path, "r")
            name_desc = []
            for line in input_file:
                name_desc.append(line.strip())
            input_file.close()
            program_description = {
                "name": name_desc[0],
                "description": name_desc[1]
            }
            program_discription_list.append(program_description)
            activity_program = get_activity_program("graph_state_list/" + scene + "/" + folder + "/" + directory + "/activityList-program.txt")
            graph_state_list = get_graph_state_list("graph_state_list/" + scene + "/" + folder + "/" + directory + "/activityList-graph-state-*.json")
            create_rdf(graph_state_list, program_description, activity_program, scene, directory)
    except Exception as e:
        print(e.args)


# # KGRC4SI

# In[14]:


def create_rdf(graph_state_list, program_description, activity_program, scene):
    base = Namespace("http://example.org/virtualhome2kg/instance/")
    onto = Namespace("http://example.org/virtualhome2kg/ontology/")
    an = Namespace("http://example.org/virtualhome2kg/ontology/action/")
    ho = Namespace("http://www.owl-ontologies.com/VirtualHome.owl#")
    x3do = Namespace("https://www.web3d.org/specifications/X3dOntology4.0#")
    time = Namespace("http://www.w3.org/2006/time#")
    g = Graph()
    g.bind("ex", base)
    g.bind("vh2kg", onto)
    g.bind("vh2kg-an", an)
    g.bind("ho", ho)
    g.bind("x3do", x3do)
    g.bind("owl", OWL)
    g.bind("time", time)
    
    init_state = graph_state_list[0]
    nodes = init_state["nodes"]
    edges = init_state["edges"]
    
    #character
    char_node = getCharNode(nodes)
    char_class_name = char_node['class_name']
    char_id = char_node['id']
    char_r = base[char_class_name + str(char_id) + "_" + scene]
    g.add((char_r, RDF.type, onto.Character))
    g.add((char_r, RDFS.label, Literal(char_class_name)))
    g.add((char_r, DCTERMS.identifier, Literal(str(char_node['id']))))
    
    #activity
    id = 0
    state_cnt = 0
    
    activity_name = program_description["name"].lower().replace(" ","_") +  "_" + scene
    activity_r = base[activity_name]
    g.add((activity_r, RDFS.label, Literal(program_description["name"])))
    g.add((activity_r, RDFS.comment, Literal(program_description["description"])))
    g.add((activity_r, RDF.type, ho[program_description["name"].lower().replace(" ","_")]))

    #event
    event_id = id
    g, event_list = getEventResourceList(g, activity_program, event_id, activity_name, char_r)
    
    for event_r in event_list:
        g.add((activity_r, onto.hasEvent, event_r))
    #event関係終了
    
    #create objects and its situations
    g, state_cnt = createObjectAndSituation(g, graph_state_list, event_list, state_cnt, activity_name, scene)
        
    #Activity
    #.add((char_r, onto.activity, activity_r))
    g.add((activity_r, onto.agent, char_r))
    scene_r = base[scene]
    g.add((scene_r, RDF.type, onto.VirtualHome))
    g.add((activity_r, onto.virtualHome, scene_r))
    
    output_path = "rdf/" + scene + "_20220909_kgrc/virtualhome2kg-" + folder.replace(" ", "_") + ".ttl"
    g.serialize(destination=output_path, format="turtle")


# In[15]:


scene = 'scene1'
folders = os.listdir("kgrc/202209/graph_state_list/" + scene + "/")
for folder in folders:
    try:
        activity_directory = "kgrc/202209/graph_state_list/" + scene + "/" + folder
        directories = os.listdir(activity_directory)
        program_discription_list = []
        program_description_path = "kgrc/202209/graph_state_list/" + scene + "/" + folder + "/program-description.txt"
        program_description = {}
        input_file = open(program_description_path, "r")
        name_desc = []
        for line in input_file:
            name_desc.append(line.strip())
        input_file.close()
        program_description = {
            "name": name_desc[0],
            "description": name_desc[1]
        }
        program_discription_list.append(program_description)
        activity_program = get_activity_program("kgrc/202209/graph_state_list/" + scene + "/" + folder + "/activityList-program.txt")
        graph_state_list = get_graph_state_list("kgrc/202209/graph_state_list/" + scene + "/" + folder + "/activityList-graph-state-*.json")
        create_rdf(graph_state_list, program_description, activity_program, scene)
    except Exception as e:
        print(e.args)


# In[ ]:





# In[ ]:




