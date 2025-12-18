import { ChatRequest, ChatResponse, ChatEvent } from './types';

// Get the API URL from the docusaurus config or default to localhost
const getApiUrl = (): string => {
  // Access custom fields from docusaurus config through window object
  if (typeof window !== 'undefined') {
    // Docusaurus injects site metadata into window object
    // Using a more reliable approach to get the custom field
    try {
      // First try to get from the global Docusaurus object
      if ((window as any).__docusaurus) {
        const config = (window as any).__docusaurus.config;
        if (config && config.customFields && config.customFields.chatbotApiUrl) {
          return config.customFields.chatbotApiUrl;
        }
      }
      
      // Fallback to default URL
      return 'http://localhost:8000/api';
    } catch (error) {
      // If accessing the global object fails, use default
      console.warn('Could not access Docusaurus config, using default API URL');
      return 'http://localhost:8000/api';
    }
  }
  
  // Fallback
  return 'http://localhost:8000/api';
};

export const chatApi = {
  // Health check endpoint
  async healthCheck(): Promise<boolean> {
    try {
      const response = await fetch(`${getApiUrl()}/health`);
      const data = await response.json();
      return data.status === 'healthy';
    } catch (error) {
      console.error('Health check failed:', error);
      return false;
    }
  },

  // Send a message and get a streaming response
  async sendMessageStream(
    message: string, 
    onEvent: (event: ChatEvent) => void,
    onError: (error: string) => void,
    conversationHistory: { role: 'user' | 'assistant'; content: string }[] = [],
    selectedText?: string | null,
    currentPage?: string | null
  ): Promise<void> {
    const request: ChatRequest = {
      message,
      selected_text: selectedText || null,
      current_page: currentPage || null,
      conversation_history: conversationHistory,
    };

    try {
      const response = await fetch(`${getApiUrl()}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      if (!response.body) {
        throw new Error('ReadableStream not available');
      }

      const reader = response.body.getReader();
      const decoder = new TextDecoder();

      let buffer = '';

      while (true) {
        const { done, value } = await reader.read();
        
        if (done) {
          break;
        }

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');
        buffer = lines.pop() || ''; // Keep last incomplete line in buffer

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            try {
              const jsonData = line.slice(6); // Remove 'data: ' prefix
              if (jsonData.trim()) {
                const event: ChatEvent = JSON.parse(jsonData);
                onEvent(event);
              }
            } catch (e) {
              console.error('Error parsing SSE data:', e);
            }
          }
        }
      }

      // Process any remaining buffered data
      if (buffer.trim()) {
        const line = buffer.trim();
        if (line.startsWith('data: ')) {
          try {
            const jsonData = line.slice(6); // Remove 'data: ' prefix
            if (jsonData.trim()) {
              const event: ChatEvent = JSON.parse(jsonData);
              onEvent(event);
            }
          } catch (e) {
            console.error('Error parsing SSE data:', e);
          }
        }
      }
    } catch (error) {
      console.error('Error sending message:', error);
      onError(error instanceof Error ? error.message : 'An unknown error occurred');
    }
  },

  // Send a message and get a synchronous response (for testing)
  async sendMessageSync(request: ChatRequest): Promise<ChatResponse> {
    try {
      const response = await fetch(`${getApiUrl()}/chat/sync`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: ChatResponse = await response.json();
      return data;
    } catch (error) {
      console.error('Error sending message:', error);
      throw error;
    }
  },
};