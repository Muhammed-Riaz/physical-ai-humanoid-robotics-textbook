export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

export interface Source {
  chapter: number;
  lesson: number;
  section: string;
  url: string;
}

export interface ChatRequest {
  message: string;
  selected_text?: string | null;
  current_page?: string | null;
  conversation_history: { role: 'user' | 'assistant'; content: string }[];
}

export interface ChatResponse {
  content: string;
  sources: Source[];
}

export interface ChatEvent {
  type: 'sources' | 'content' | 'done' | 'error';
  content?: string;
  sources?: Source[];
  chunk?: string;
}

export interface ChatHistory {
  messages: Message[];
}