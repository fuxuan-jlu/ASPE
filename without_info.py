from prompts import Prompts
from evaluate import Evaluate
import code_init
import random


class ASPE_Interface:
    def __init__(self, size, api_endpoint, api_key, model_LLM, debug_mode):
        self.size = size  # Population size
        self.prompts = Prompts(api_endpoint, api_key, model_LLM, debug_mode)
        self.population = []
        self.generation = 0
        self.code_init = code_init.CodeInit()  # Initialize code_init instance

    def initialize_population(self):
        """Initialize population using the multi-agent system to mutate the initial code"""
        initial_code = self.code_init.get_initial_code()  # Get initial code from code_init instance
        for _ in range(self.size):
            # Use the multi-agent system to generate a variant of the initial code
            new_individual = self.generate_new_individual(initial_code)
            self.population.append(new_individual)

    def generate_new_individual(self, base_code):
        """Use the multi-agent system to generate a new individual"""
        try:
            # Step 1: Code understanding - get sub-problems and their algorithms
            prompt1 = self.prompts._get_sub_prob("FSTSP", base_code)
            response1 = self.prompts.interface_llm.get_response(prompt1)
            sub_prob_alg = self.prompts._extract_sub_problems(response1)

            # Step 2: Select sub-problem to improve
            prompt2 = self.prompts._get_select_prob("FSTSP", base_code, sub_prob_alg)
            response2 = self.prompts.interface_llm.get_response(prompt2)
            sub_problem = self.prompts._extract_sub_problem(response2)

            # Step 3: Algorithm improvement
            prompt3 = self.prompts._get_alg_imp("FSTSP", base_code, sub_prob_alg, sub_problem)
            response3 = self.prompts.interface_llm.get_response(prompt3)
            new_algorithm = self.prompts._extract_new_algorithm(response3)

            # Step 4: Code improvement
            prompt4 = self.prompts._get_code_imp("FSTSP", base_code, sub_prob_alg, sub_problem, new_algorithm)
            response4 = self.prompts.interface_llm.get_response(prompt4)
            new_code = self.prompts._extract_code(response4)

            # Evaluate the new code
            score = Evaluate(new_code)

            return {'code': new_code, 'score': score}

        except Exception as e:
            # If there's an error, use the debug agent to fix the code
            print(f"Error in generation process: {e}")
            prompt_debug = self.prompts._get_code_debug("FSTSP", base_code, str(e))
            response_debug = self.prompts.interface_llm.get_response(prompt_debug)
            fixed_code = self.prompts._extract_code(response_debug)
            score = Evaluate(fixed_code)
            return {'code': fixed_code, 'score': score}


    def evolve(self, m ,maxsize):
        """Run the evolutionary process for m generations"""
        if not self.population:
            self.initialize_population()

        for generation in range(m):
            self.generation += 1

            # Generate new individuals
            while len(self.population) < maxsize:
                # Select parent (tournament selection)
                parent = random.choice(self.population)  # Select from top 50%

                # Generate offspring using the multi-agent system
                offspring = self.generate_new_individual(parent['code'])
                self.population.append(offspring)

            self.population.sort(key=lambda x: x['score'], reverse=False)
            self.population = self.population[:self.size]

            print(f"Generation {self.generation}: Best score = {self.population[0]['score']}")

    def get_best_solution(self):
        """Return the best solution found"""
        if not self.population:
            return None
        self.population.sort(key=lambda x: x['score'], reverse=False)
        return self.population[0]

import ast
import json
import re


class Prompts:
    def __init__(self, api_endpoint, api_key, model_LLM, debug_mode):
        # 初始化LLM接口
        self.interface_llm = InterfaceLLM(
            api_endpoint, api_key,
            model_LLM,debug_mode
        )

    def _get_sub_prob(self,prob_name,code):
        #代码理解提示词
        prompt = f"""
                code：
                {code}
                The above is a solution program for an {prob_name} problem. Think about what sub-problems the program solves and return in the format:
                [{{Sub_problem: …, Solving_Algorithm: ...}}, …] 
                Do not make extra explanation.
                """

        return prompt

    def _get_select_prob(self,prob_name,code, sub_prob_alg):
        #子问题选择提示词
        prompt = f"""
                code：
                {code}
                The above is a solution program for an {prob_name} problem. The program solves the following sub-problems: { sub_prob_alg }
                 Please choose the sub-problem whose solving algorithm needs the most improvement. By improving its algorithm, the program's running efficiency and solution quality can be maximized. 
                 Only provide the corresponding sub-problem in the form: {{Sub_problem: …}}, 
                 Do not make extra explanation.
                """

        return prompt

    def _get_alg_imp(self,prob_name,code, sub_prob_alg,sub_problem):
        #算法改进提示词
        prompt = f"""
                code：
                {code}
                The above is a solution program for an {prob_name} problem.
                 The program solves the following sub-problems: {sub_prob_alg}
                Please improve the solving algorithm for the sub-problem: {sub_problem} to enhance its running efficiency. 
                Return in the form of {{New_Algorithm: …}}, where "…" is a natural language description of the new algorithm. 
                The algorithm should be concise and different with the original algorithm. Do not make extra explanation.
                """

        return prompt

    def _get_code_imp(self,prob_name,code, sub_prob_alg,sub_problem,New_Algorithm):
        #代码更新提示词
        prompt = f"""
                code：
                {code}
                The above is a solution program for an {prob_name} problem. The program solves the following sub-problems: { sub_prob_alg }
                Among them, for {sub_problem}, an improved solving algorithm has been proposed: {New_Algorithm}. 
                Improve the program by using the new algorithm to solve the corresponding sub-problem, and provide the complete improved program.
                """

        return prompt

    def _get_code_debug(self,prob_name,code,error):
        #代码纠错提示词
        prompt = f"""
                code：
                {code}
                The above is a solution program for an {prob_name} problem. 
                The program encountered the following error: {error}.
                 Modify the code based on the error message and provide the corrected complete code.
                """

        return prompt

    # 新增数据提取方法
    def _extract_sub_problems(self, response):
        """
        从LLM响应中提取子问题列表
        目标格式: [{Sub_problem: ..., Solving_Algorithm: ...}, ...]
        """
        try:
            # 尝试直接解析JSON格式
            if response.strip().startswith('['):
                return json.loads(response)

            # 尝试解析Python字面量
            pattern = r'\[.*?\]'
            match = re.search(pattern, response, re.DOTALL)
            if match:
                return ast.literal_eval(match.group(0))

            # 备用模式匹配
            pattern = r'\{.*?\}'
            items = re.findall(pattern, response, re.DOTALL)
            if items:
                return [ast.literal_eval(item) for item in items]

        except (json.JSONDecodeError, SyntaxError) as e:
            print(f"解析错误: {e}")

        # 最终回退方案
        print("使用回退解析方案")
        results = []
        pattern = r'Sub_problem:\s*"(.*?)",\s*Solving_Algorithm:\s*"(.*?)"'
        matches = re.findall(pattern, response)
        for match in matches:
            results.append({
                "Sub_problem": match[0],
                "Solving_Algorithm": match[1]
            })
        return results

    def _extract_sub_problem(self, response):
        """
        从LLM响应中提取选定的子问题
        目标格式: {Sub_problem: ...}
        """
        try:
            # 尝试直接解析JSON格式
            if response.strip().startswith('{'):
                return json.loads(response)

            # 尝试解析Python字面量
            pattern = r'\{.*?\}'
            match = re.search(pattern, response, re.DOTALL)
            if match:
                return ast.literal_eval(match.group(0))
        except (json.JSONDecodeError, SyntaxError) as e:
            print(f"解析错误: {e}")

        # 备用方案：提取Sub_problem字段
        print("使用回退解析方案")
        match = re.search(r'Sub_problem:\s*"(.*?)"', response)
        if match:
            return {"Sub_problem": match.group(1)}
        return {"Sub_problem": response.strip()}

    def _extract_new_algorithm(self, response):
        """
        从LLM响应中提取新算法描述
        目标格式: {New_Algorithm: ...}
        """
        try:
            # 尝试直接解析JSON格式
            if response.strip().startswith('{'):
                return json.loads(response)

            # 尝试解析Python字面量
            pattern = r'\{.*?\}'
            match = re.search(pattern, response, re.DOTALL)
            if match:
                return ast.literal_eval(match.group(0))
        except (json.JSONDecodeError, SyntaxError) as e:
            print(f"解析错误: {e}")

        # 备用方案：提取New_Algorithm字段
        print("使用回退解析方案")
        match = re.search(r'New_Algorithm:\s*"(.*?)"', response)
        if match:
            return {"New_Algorithm": match.group(1)}
        return {"New_Algorithm": response.strip()}

    def _extract_code(self, response):
        """
        从LLM响应中提取Python代码
        处理三种常见格式：带标记的代码块、纯代码、包含代码的文本
        """
        # 尝试提取带标记的代码块
        code_block_pattern = r'```python(.*?)```'
        match = re.search(code_block_pattern, response, re.DOTALL)
        if match:
            return match.group(1).strip()

        # 尝试提取无标记的代码块
        no_marker_pattern = r'```(.*?)```'
        match = re.search(no_marker_pattern, response, re.DOTALL)
        if match:
            return match.group(1).strip()

        # 最终回退：返回整个响应
        return response.strip()
