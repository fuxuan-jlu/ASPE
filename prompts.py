import ast
import json
import re
from interface_llm import InterfaceLLM

class Prompts:
    def __init__(self, api_endpoint, api_key, model_LLM, debug_mode):
        self.interface_llm = InterfaceLLM(
            api_endpoint, api_key,
            model_LLM, debug_mode
        )

    def get_sub_prob(self, prob_name, program_info):
        """改进后的提示词"""
        prompt = f"""
        You MUST respond in valid JSON format ONLY. Example:
           [
                {{
                    "Sub_problem": "description", 
                    "Solving_Algorithm": "algorithm"
                }} , ...
            ]

        Analyze this {prob_name} solution code:
        {program_info['code']}

        Identify sub-problems and their algorithms. Return ONLY the JSON object.
        """
        return self.interface_llm.get_response(prompt)

    def get_select_prob(self, prob_name, program_info):
        """Sub-problem selection prompt"""
        prompt = f"""
        code:
        {program_info['code']}
        The above is a solution program for an {prob_name} problem. The program solves the following sub-problems: {program_info['sub_prob_alg']}
        Please choose the sub-problem whose solving algorithm needs the most improvement. By improving its algorithm, the program's running efficiency and solution quality can be maximized.
        Return in the form: {{"..."}}
        The part "..." must be a nature language description of the sub-problem
        Do not make extra explanation.
        """
        return self.interface_llm.get_response(prompt)

    def get_alg_imp(self, prob_name, program_info):
        """Algorithm improvement prompt"""
        prompt = f"""
        code:
        {program_info['code']}
        The above is a solution program for an {prob_name} problem.
        The program solves the following sub-problems: {program_info['sub_prob_alg']}
        Please improve the solving algorithm for the sub-problem: {program_info['sub_problem']} to enhance its running efficiency.
        Return in the form: {{"..."}}
        The part "..." must be a nature language description of new algorithm
        The new algorithm should be concise and different with the original algorithm. 
        Do not make extra explanation.
        """
        return self.interface_llm.get_response(prompt)

    def get_code_imp(self, prob_name, program_info):
        """Code update prompt"""
        #No escape symbols are allowed in the code.
        prompt = f"""
        code:
        {program_info['code']}
        The above is a solution program for an {prob_name} problem. The program solves the following sub-problems: {program_info['sub_prob_alg']}
        Among them, for {program_info['sub_problem']}, an improved solving algorithm has been proposed: {program_info['new_algorithm']}.
        Improve the program by using the new algorithm to solve the corresponding sub-problem, and provide the complete improved program.
        Ensure the code includes ALL necessary imports.
        Return in the form: {{"..."}}
        The part "..." must be the complete improved program
        Do not make extra explanation.
        """
        return self.interface_llm.get_response(prompt)

    def get_code_debug(self, prob_name, program_info):
        """Code debugging prompt"""
        prompt = f"""
        code:
        {program_info['code']}
        The above is a solution program for an {prob_name} problem.
        The program encountered the following error: {program_info['error']}.
        Modify the code based on the error message and provide the corrected complete code.
        Return in the form: {{"..."}}
        The part "..." must be the corrected complete program
        Do not make extra explanation.
        """
        return self.interface_llm.get_response(prompt)

    def extract_sub_problems(self, response):
        """Extract all sub-problems and their corresponding solving algorithms"""
        if not isinstance(response, str):
            print(f"Error: response must be a string, got {type(response)}")
            return {'sub_prob_alg': []}

        # Remove Markdown code block markers if present
        cleaned_response = response.strip()
        if cleaned_response.startswith('```json') and cleaned_response.endswith('```'):
            cleaned_response = cleaned_response[7:-3].strip()  # Remove ```json and ```
        elif cleaned_response.startswith('```') and cleaned_response.endswith('```'):
            cleaned_response = cleaned_response[3:-3].strip()  # Remove generic ```

        try:
            data = json.loads(cleaned_response)

            if not isinstance(data, list):
                print("Error: Expected JSON array, got", type(data))
                return {'sub_prob_alg': []}

            # Validate and extract each item
            sub_prob_alg = []
            for item in data:
                if not isinstance(item, dict):
                    continue
                sub_problem = item.get("Sub_problem")
                algorithm = item.get("Solving_Algorithm")
                if sub_problem and algorithm:
                    sub_prob_alg.append({
                        "Sub_problem": sub_problem,
                        "Solving_Algorithm": algorithm
                    })

            return {'sub_prob_alg': sub_prob_alg}

        except json.JSONDecodeError as e:
            print(f"JSON Decode Error: {e}\nResponse content:\n{cleaned_response}")
            return {'sub_prob_alg': []}
        except Exception as e:
            print(f"Unexpected Error: {e}")
            return {'sub_prob_alg': []}

    def extract_sub_problem(self, response):
        """Extract selected sub-problem from a JSON response."""
        try:
            # 使用正则表达式匹配花括号及其内部内容
            match = re.search(r'\{.*?\}', response, re.DOTALL)
            if match:
                return {'sub_problem': match.group(0)}
            else:
                return None  # 如果没有找到匹配项，返回 None
        except Exception as e:
            print(f"Error extracting sub-problem: {e}")
            return None

    def extract_new_algorithm(self, response):
        """Extract new algorithm into program_info"""
        try:
            match = re.search(r'\{.*?\}', response, re.DOTALL)
            if match:
                try:
                    return {'new_algorithm':match.group(0)}  # 返回 dict 对象
                except json.JSONDecodeError as je:
                    print(f"Invalid JSON: {je}")
                    return None
            else:
                return None
        except Exception as e:
            print(f"Error extracting new algorithm: {e}")
            return None

    def extract_code(self, response):
        """从包含 JSON 代码块的响应中提取代码"""

        try:
            match = re.search(r'{(.*)}', response, re.DOTALL)
            if match:
                print("good!")
                pos = match.group(1).find('import')
                match = match.group(1)[pos:]
                return {'code': match}
            else:
                match = re.search(r'(import.*[\'"]$)', response, re.DOTALL)
                if match:
                    return {'code': match.group(0)}
                else:
                    print("在响应中未找到匹配的模式")
                    return {'code': ''}  # 如果没有匹配，返回空的代码字典
        except Exception as e:
            print(f"错误: {e}")
            return {'code': ''}  # 其他错误的后备处理


"""
        try:
            # Use regex to extract the content within the ```python block
            match = re.search(r'```python\n([\s\S]*?)\n```', response, re.DOTALL)
            if not match:
                print("Error: No python code block found in response")
                return {'code': ''}

            # Extract the raw content
            code_content = match.group(1)

            # Try to parse it as JSON
            try:
                data = json.loads(code_content)
                return {'code': data.get('')}
            except json.JSONDecodeError as e:
                # If JSON parsing fails, assume the content is the code itself
                print(f"JSON Error: {e} - Treating content as raw code")
                return {'code': code_content.strip()}
        except Exception as e:
            print(f"Error: {e}")
            return {'code': ''}
"""