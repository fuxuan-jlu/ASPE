from api_general import InterfaceAPI


class InterfaceLLM:
    def __init__(self, api_endpoint, api_key, model_LLM, debug_mode):
        self.api_endpoint = api_endpoint
        self.api_key = api_key
        self.model_LLM = model_LLM
        self.debug_mode = debug_mode


        print('remote llm api is used ...')

        if self.api_key == None or self.api_endpoint == None or self.api_key == 'xxx' or self.api_endpoint == 'xxx':
            print(
                ">> Stop with wrong API setting: Set api_endpoint (e.g., api.chat...) and api_key (e.g., kx-...) !")
            exit()

        self.interface_llm = InterfaceAPI(
            self.api_endpoint,
            self.api_key,
            self.model_LLM,
            self.debug_mode,
        )

        res = self.interface_llm.get_response("1+1=?")

        if res == None:
            print(">> Error in LLM API, wrong endpoint, key, model or local deployment!")
            exit()

        # choose LLMs
        # if self.type == "API2D-GPT":
        #     self.interface_llm = InterfaceAPI2D(self.key,self.model_LLM,self.debug_mode)
        # else:
        #     print(">>> Wrong LLM type, only API2D-GPT is available! \n")

    def get_response(self, prompt_content):
        response = self.interface_llm.get_response(prompt_content)

        return response