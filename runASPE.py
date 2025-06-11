from aspe_interface import ASPE_Interface
import os

def main():
    # Define all parameters in a dictionary
    paras = {
        'size': 3,  # Population size
        "api_endpoint": "https://ark.cn-beijing.volces.com/api/v3",
        "api_key": os.environ.get("ARK_API_KEY"),  # 修正括号位置
        "model_LLM": "deepseek-v3-250324",
        'debug_mode': False,  # Set to False in production
        'generations': 5,  # Number of generations to evolve
        'max_pop_size': 6  # Maximum population size during evolution
    }

    # Initialize ASPE interface with parameters
    aspe = ASPE_Interface(
        size=paras['size'],
        api_endpoint=paras['api_endpoint'],
        api_key=paras['api_key'],
        model_LLM=paras['model_LLM'],
        debug_mode=paras['debug_mode']
    )

    # Run the evolutionary process
    aspe.evolve(m=paras['generations'], maxsize=paras['max_pop_size'])

    # Get and display the best solution
    best_solution = aspe.get_best_solution()
    print("\nBest Solution Found:")
    print(f"Score: {best_solution['score']}")
    print(f"Code:\n{best_solution['code']}")


if __name__ == "__main__":
    main()