#!/usr/bin/env python
# coding: utf-8

# In[1]:


#get_ipython().run_line_magic('matplotlib', 'notebook')
#import IPython.display
from utils_demo import *
from sys import platform
import sys
from PIL import Image
import matplotlib.pyplot as plt
import json
import rdflib
import glob
import os
import re
import copy
import time

sys.path.append('../simulation')
sys.path.append('../dataset_utils/')

import numpy as np
import random
import cv2
import add_preconds
import evolving_graph.check_programs as check_programs
import evolving_graph.utils as utils

from unity_simulator.comm_unity import UnityCommunication


# In[2]:


comm = UnityCommunication()


# In[3]:


scene = 1
scene_graph = "TrimmedTestScene" + str(scene) + "_graph"
executable_program_path = "../dataset/programs_processed_precond_nograb_morepreconds/executable_programs/" + scene_graph + "/*/*.txt"
executable_program_list = []
for file_path in glob.glob(executable_program_path):
    executable_program_list.append(file_path.replace("../dataset/programs_processed_precond_nograb_morepreconds/executable_programs/" + scene_graph + "/", ""))


# In[4]:


rdf_g = rdflib.Graph()
rdf_g.parse("../ontology/vh2kg_ontology.ttl", format="ttl")


# In[5]:


def get_activity_from_ontology(activity_type):
    results = []
    qres = rdf_g.query(
    """
PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
PREFIX : <http://www.owl-ontologies.com/VirtualHome.owl#>
PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
select ?activity where { 
    ?activity rdfs:subClassOf :%s .
 } 
       """ % activity_type)

    result = ""
    for row in qres:
        activity = "".join(row).replace("http://www.owl-ontologies.com/VirtualHome.owl#","")
        arr = activity.split("_")
        arr[0] = arr[0].capitalize()
        activity = " ".join(arr)
        result = activity
        results.append(result)
    return results


# In[6]:


# set character
program_list = []
comm.reset(scene-1)
comm.add_character('chars/Female2')


# In[7]:


# load environment
success, g = comm.environment_graph();


# In[23]:


#nodes = g["nodes"]
#edges = g["edges"]
#max_id = nodes[len(nodes)-1]["id"]


# In[8]:


def generate_list_of_steps(file_path):
    file = open(file_path, "r", encoding="utf-8")
    i = 0
    list_of_steps = []
    program_name = ""
    description = ""
    char= "<char0>"
    while True:
        line = file.readline()
        if line:
            line = line.replace("\n","")
            if i==0:
                program_name = line
            elif i==1:
                description = line
            elif line.startswith("["):
                list_of_steps.append(line)
            else:
                pass
            i+=1
        else:
            break
    return program_name, description, list_of_steps


# In[9]:


data_path = "../dataset/programs_processed_precond_nograb_morepreconds/withoutconds/*/*.txt"
program_list = []
for file_path in glob.glob(data_path):
    file_name = file_path.replace("../dataset/programs_processed_precond_nograb_morepreconds/withoutconds/", "")
    if file_name in executable_program_list:
        program_name, description, list_of_steps = generate_list_of_steps(file_path)
        program_list.append({
            "file_name":file_name,
            "name": program_name,
            "description": description,
            "list_of_steps": list_of_steps
        })


# In[10]:


def get_activity_program(category):
    #unexecutable = ["Take shower", "Take shoes off", "Wash teeth", "Wash face", "Dust", "Clean toilet", "Clean room", "Scrubbing living room tile floor is once week activity for me", "Clean mirror", "Play games", "Play on laptop", "Read on sofa"]
    unexecutable = []
    executable = []
    activities = get_activity_from_ontology(category)
    for activity_name in activities:
        if activity_name in unexecutable:
            continue
        results = [program for program in program_list if program["name"] == activity_name]
        if len(results) == 0:
            print("Nothing: " + activity_name)
        else:
            print("Success: " + activity_name)
            executable.append({"activity_name": activity_name, "results": results})
    return executable


# In[11]:


def delete2000(graph_state_list):
    new_graph_state_list = []
    for graph_state in graph_state_list:
        new_nodes = [x for x in graph_state["nodes"] if x["id"] < 2000]
        new_edges = [x for x in graph_state["edges"] if x["from_id"] < 2000 and x["to_id"] < 2000]
        new_graph_state_list.append({"nodes": new_nodes, "edges": new_edges})
    return new_graph_state_list


# In[12]:


def export(activity_name, graph_state_list, activity_cnt):
    os.mkdir("graph_state_list/scene" + str(scene) + "/" + activity_name + "/" + activity_cnt)
    state_cnt = 0
    for graph_state in graph_state_list:
        state_cnt += 1
        file_path = "graph_state_list/scene" + str(scene) + "/"  + activity_name + "/" + activity_cnt + "/activityList-graph-state-" + '{0:03d}'.format(state_cnt) + ".json"
        with open(file_path, 'w') as outfile:
            json.dump(graph_state, outfile)

    with open("graph_state_list/scene" + str(scene) + "/" + activity_name + "/" + activity_cnt + "/activityList-program.txt", 'w') as f:
        for s in executed_program:
            f.write("%s\n" % s)

    with open("graph_state_list/scene" + str(scene) + "/" + activity_name + "/" + activity_cnt + "/program-description.txt", 'w') as f:
        f.write("%s\n" % activity["name"])
        f.write("%s\n" % activity["description"])


# In[13]:


executable_activity_list = []
for activity_class in ["BedTimeSleep", "EatingDrinking", "FoodPreparation", "GettingReady", "HouseArrangement", "HouseCleaning", "HygieneStyling", "Leisure", "Other", "PhysicalActivity", "SocialInteraction", "Work"]:
    executable_activity_list.extend(get_activity_program(activity_class))


# In[16]:


for executable_activity in executable_activity_list:
    activity_list = executable_activity["results"]
    activity_name = executable_activity["activity_name"]
    print(activity_name+"\n")
    activity_cnt = 0
    try:
        os.makedirs("graph_state_list/scene" + str(scene) + "/" + activity_name,exist_ok=True)
        for activity in activity_list:
            try:
                comm.reset(scene-1)
                comm.add_character('chars/Female2')
                success, g = comm.environment_graph();
                print(activity["file_name"])
                script = activity["list_of_steps"]
                preconds = add_preconds.get_preconds_script(script).printCondsJSON()
                g = check_programs.translate_graph_dict_nofile(g)
                info = check_programs.check_script(script, preconds, graph_path=None, inp_graph_dict=g)
                message, final_state, graph_state_list, graph_dict, id_mapping, info, helper, executed_script = info
                if message != "Script is executable":
                    print(message)
                    print("skipped")
                    continue
                # if delete randomly placced objects 
                graph_state_list = delete2000(graph_state_list)
                print(message)
                executed_program = []
                for x in executed_script:
                    executed_program.append("<char0> " + re.sub("\[\d+\]","",x.__str__()).strip())

                export(activity_name, graph_state_list, str(activity_cnt))
                activity_cnt += 1
            except Exception as e:
                print(e.args)
    except Exception as e:
        print(e.args)


# In[ ]:




