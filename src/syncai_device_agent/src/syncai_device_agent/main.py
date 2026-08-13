import structlog
from deepagents import create_deep_agent

class DeviceAgent:
    
    def __init__(self):
        self.logger = structlog.get_logger()
        self.logger.info("Initializer device agent")

        internet_search = {"type": "web_search_20260209", "name": "web_search"}
        instructions = """You are an expert researcher. Your job is to conduct thorough research and then write a polished report.
        
        You have access to an internet search tool as your primary means of gathering information.
        
        ## `internet_search`
        
        Use this to run an internet search for a given query. You can specify the max number of results to return, the topic, and whether raw content should be included.
        """

        self.agent = create_deep_agent(
            model="anthropic:claude-sonnet-4-6",
            tools=[internet_search],
            system_prompt=instructions,
        )

    def run(self):

        result = self.agent.invoke({"messages": [{"role": "user", "content": "What is langgraph?"}]})

        print(result["messages"][-1]["content"])


if __name__ == "__main__":
    device_agent = DeviceAgent()
    device_agent.run()