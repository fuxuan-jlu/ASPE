import sys
from copy import deepcopy

from prompts import Prompts
from evaluate import Evaluate
import code_init
import json
import ast
import re
import traceback
import random
import time
from io import StringIO
import contextlib

class ASPE_Interface:
    def __init__(self, size, api_endpoint, api_key, model_LLM, debug_mode):
        self.size = size  # Population size
        self.prompts = Prompts(api_endpoint, api_key, model_LLM, debug_mode)
        self.population = []  # Will store dictionaries with program_info
        self.generation = 0
        self.code_init = code_init.CodeInit()  # Initialize code_init instance

    def initialize_population(self):
        """Initialize population using the multi-agent system to mutate the initial code"""
        initial_code = self.code_init.get_initial_code()  # Get initial code from code_init instance
        program_info = {'code': initial_code}
        program_info['score'] = Evaluate(program_info['code'])
        self.population.append(program_info)
        """
        for _ in range(self.size):
            # Initialize program_info with the base code
            # Use the multi-agent system to generate a variant of the initial code
            new_individual = self.generate_new_individual(program_info)
            self.population.append(new_individual)
        """
    def generate_new_individual(self, program_info):
        """Use the multi-agent system to generate a new individual"""
        try:
            # Step 1: Code understanding - get sub-problems and their algorithms
            response1 = self.prompts.get_sub_prob("FSTSP", program_info)
            #print(response1)
            program_info.update(self.prompts.extract_sub_problems(response1))

            # Step 2: Select sub-problem to improve
            response2 = self.prompts.get_select_prob("FSTSP", program_info)
            #print(response2)
            program_info.update(self.prompts.extract_sub_problem(response2))

            # Step 3: Algorithm improvement
            response3 = self.prompts.get_alg_imp("FSTSP", program_info)
            #print(response3)
            program_info.update(self.prompts.extract_new_algorithm(response3))

            # Step 4: Code improvement
            response4 = self.prompts.get_code_imp("FSTSP", program_info)
            #print(response4)
            program_info.update(self.prompts.extract_code(response4))
            #print(program_info)
            # Evaluate the new code

            program_info['score'] = Evaluate(program_info['code'])
            print("yes!")
            return program_info

        except Exception as e:
            m = 0
            # If there's an error, use the debug agent to fix the code
            while m < 5:
                try:
                    exc_type, exc_value, exc_traceback = sys.exc_info()
                    # Get full stack trace as string
                    full_traceback = "".join(traceback.format_exception(exc_type, exc_value, exc_traceback))
                    print(f"Error in generation process: {full_traceback}")
                    program_info['error'] = full_traceback  # Store complete traceback instead of just error message
                    #print(f"Error in generation process: {str(e)}")
                    #program_info['error'] = str(e)
                    response_debug = self.prompts.get_code_debug("FSTSP", program_info)
                    program_info.update(self.prompts.extract_code(response_debug))
                    program_info['score'] = Evaluate(program_info['code'])
                    return program_info
                except Exception as e:
                    m += 1
            program_info['score'] = 1000
            return program_info


    def evolve(self, m, maxsize):
        """Run the evolutionary process for m generations"""
        if not self.population:
            self.initialize_population()

        for generation in range(m):
            self.generation += 1

            # Generate new individuals
            while len(self.population) < maxsize:
                # Select parent (tournament selection)
                parent = random.choice(self.population[:len(self.population)//2+1])  # Select from top 50%
                # Generate offspring using the multi-agent system
                offspring = self.generate_new_individual(deepcopy(parent))
                #print("yes!")
                print(offspring)
                self.population.append(offspring)

            # Sort and select the best individuals
            self.population.sort(key=lambda x: x['score'], reverse=False)
            self.population = self.population[:self.size]

            print(f"Generation {self.generation}: Best score = {self.population[0]['score']}")

    def get_best_solution(self):
        """Return the best solution found"""
        if not self.population:
            return None
        self.population.sort(key=lambda x: x['score'], reverse=False)
        return self.population[0]

