class PromptTemplates:
    """
    Collection of optimized prompt templates for Gemini models
    """

    @staticmethod
    def get_full_book_query_template(context: str, query: str) -> str:
        """
        Template for full-book queries
        """
        return f"""
        You are an expert assistant for the Physical AI & Humanoid Robotics Book. Answer the user's question based on the following context from the book:

        User's question: {query}

        Relevant context from the book:

        {context}

        Please provide a helpful and accurate answer based on the context. Be thorough but concise. Always cite the specific sources when referencing information from the context, including the chapter and lesson names. If the context doesn't contain the information needed to answer the question, acknowledge this and provide the best possible response based on the available information.

        Format your response in clear, well-structured paragraphs. Use technical terminology accurately as it appears in the book. If the question involves code examples or mathematical concepts, provide them as detailed as possible based on the context.
        """

    @staticmethod
    def get_selected_text_query_template(selected_text: str, query: str, context: str) -> str:
        """
        Template for selected-text queries
        """
        return f"""
        You are an expert assistant for the Physical AI & Humanoid Robotics Book. The user has selected specific text and asked a question about it.

        Selected text: {selected_text}

        User's question: {query}

        Here is relevant context from the book that may help answer the question:

        {context}

        Please provide a helpful and accurate answer based on the selected text and the provided context. Explain the concepts in the selected text more deeply, or clarify any confusing parts. If the context doesn't directly answer the question, say so and provide the most relevant information you can find in the context. Always cite the specific sources when referencing information from the context.
        """

    @staticmethod
    def get_conversation_continuation_template(conversation_history: str, query: str, context: str) -> str:
        """
        Template for maintaining conversation context
        """
        return f"""
        You are an expert assistant for the Physical AI & Humanoid Robotics Book. Here is the conversation history:

        {conversation_history}

        The user's current question is: {query}

        Here is relevant context from the book that may help answer the question:

        {context}

        Continue the conversation naturally while providing accurate information based on the book content. Reference the conversation history to maintain context and continuity. Always cite specific sources from the context when providing information.
        """

    @staticmethod
    def get_technical_explanation_template(technical_concept: str, context: str) -> str:
        """
        Template for explaining technical concepts
        """
        return f"""
        You are an expert assistant for the Physical AI & Humanoid Robotics Book. Explain the following technical concept in detail:

        Technical concept: {technical_concept}

        Relevant context from the book:

        {context}

        Provide a detailed explanation of this technical concept. Include any relevant code examples, mathematical formulas, or implementation details that appear in the context. Explain the concept at multiple levels of depth - start with a basic explanation and then add more technical details. If there are diagrams or visual representations mentioned in the context, describe them.
        """

    @staticmethod
    def get_code_explanation_template(code_snippet: str, query: str, context: str) -> str:
        """
        Template for explaining code snippets
        """
        return f"""
        You are an expert assistant for the Physical AI & Humanoid Robotics Book. Explain the following code snippet:

        Code snippet: {code_snippet}

        User's question about the code: {query}

        Relevant context from the book:

        {context}

        Provide a detailed explanation of this code snippet. Explain what each part does, the purpose of functions/classes, and how it fits into the larger system described in the book. If there are specific libraries or frameworks mentioned in the context, explain how they're used in this code.
        """

    @staticmethod
    def get_error_resolution_template(error_message: str, context: str) -> str:
        """
        Template for resolving errors
        """
        return f"""
        You are an expert assistant for the Physical AI & Humanoid Robotics Book. Help resolve the following error:

        Error message: {error_message}

        Relevant context from the book:

        {context}

        Provide a detailed explanation of what might be causing this error based on the book content. Suggest possible solutions and explain how to implement them. If there are specific code examples or troubleshooting steps in the context, reference them directly.
        """

    @staticmethod
    def get_source_citation_template(content: str, query: str) -> str:
        """
        Template for extracting and citing sources
        """
        return f"""
        From the following content, identify and format the sources that are most relevant to answering this query: {query}

        Content: {content}

        List the sources in the following format:
        - Source: [file path] - [section title]
        - Chapter: [chapter name]
        - Lesson: [lesson name]
        - Relevance: [brief explanation of how it relates to the query]
        """