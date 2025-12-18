import React, { useState, useEffect, useRef } from 'react';
import { chatApi } from './api';
import { Message, Source, ChatEvent } from './types';
import styles from './styles.module.css';

interface ChatWidgetProps {
  initialIsOpen?: boolean;
}

const ChatWidget: React.FC<ChatWidgetProps> = ({ initialIsOpen = false }) => {
  const [isOpen, setIsOpen] = useState(initialIsOpen);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [sources, setSources] = useState<Source[]>([]);
  const [error, setError] = useState<string | null>(null);
  
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  // Function to scroll to the bottom of messages
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Effect to scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Effect to get selected text from page
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Function to handle sending a message
  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message to the chat
    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    const currentMessages = [...messages, userMessage];
    setInputValue('');
    setIsLoading(true);
    setError(null);
    setSources([]); // Clear previous sources

    try {
      // Get the current page URL
      const currentPage = window.location.pathname;

      // Call the API with streaming
      await chatApi.sendMessageStream(
        userMessage.content,
        (event: ChatEvent) => {
          if (event.type === 'sources' && event.sources) {
            setSources(event.sources);
          } else if (event.type === 'content' && event.chunk) {
            // Add assistant message or update existing one
            setMessages(prev => {
              const lastMessage = prev[prev.length - 1];
              if (lastMessage && lastMessage.role === 'assistant') {
                // Update existing assistant message
                const updatedMessages = [...prev];
                updatedMessages[updatedMessages.length - 1] = {
                  ...lastMessage,
                  content: lastMessage.content + event.chunk
                };
                return updatedMessages;
              } else {
                // Create new assistant message
                return [
                  ...prev,
                  {
                    id: `assistant-${Date.now()}`,
                    role: 'assistant',
                    content: event.chunk,
                    timestamp: new Date(),
                  }
                ];
              }
            });
          } else if (event.type === 'done') {
            setIsLoading(false);
          } else if (event.type === 'error') {
            setError(event.content || 'An error occurred');
            setIsLoading(false);
          }
        },
        (error: string) => {
          setError(error);
          setIsLoading(false);
          
          // Add error message to chat
          setMessages(prev => [
            ...prev,
            {
              id: `error-${Date.now()}`,
              role: 'assistant',
              content: 'Sorry, I encountered an error while processing your request.',
              timestamp: new Date(),
            }
          ]);
        },
        currentMessages.map(msg => ({ role: msg.role, content: msg.content })),
        selectedText,
        currentPage
      );
    } catch (err) {
      setError('Failed to send message');
      setIsLoading(false);
      console.error('Error sending message:', err);
    }
  };

  // Handle key press for sending message
  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Toggle chat widget open/closed
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Close chat widget
  const closeChat = () => {
    setIsOpen(false);
  };

  return (
    <div className={styles.chatWidgetContainer}>
      {/* Floating Button */}
      <button
        className={`${styles.floatingButton} ${isOpen ? styles.open : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        {isOpen ? "Chat" : "🤖"}
      </button>

      {/* Chat Panel */}
      <div className={`${styles.chatPanel} ${isOpen ? styles.open : ''}`}>
        {/* Chat Header */}
        <div className={styles.chatHeader}>
          <h3>Physical AI Chatbot</h3>
          <button 
            className={styles.closeButton} 
            onClick={closeChat}
            aria-label="Close chat"
          >
            ×
          </button>
        </div>

        {/* Messages Container */}
        <div className={styles.messagesContainer}>
          {messages.length === 0 ? (
            <div className={styles.emptyState}>
              <div className={styles.emptyStateIcon}>🤖</div>
              <p className={styles.emptyStateText}>
                Ask me anything about Physical AI & Humanoid Robotics!
              </p>
            </div>
          ) : (
            <>
              {messages.map((message) => (
                <div
                  key={message.id}
                  className={`${styles.message} ${
                    message.role === 'user' ? styles.userMessage : styles.assistantMessage
                  }`}
                >
                  {message.content}
                  
                  {/* Show sources for assistant messages if available */}
                  {message.role === 'assistant' && sources.length > 0 && (
                    <div className={styles.sourcesContainer}>
                      <strong>Sources:</strong>
                      {sources.map((source, index) => (
                        <div key={index} className={styles.sourceItem}>
                          <span>
                            Ch {source.chapter}, L {source.lesson}: {source.section}
                          </span>
                          <a 
                            href={source.url} 
                            className={styles.sourceLink}
                            target="_blank"
                            rel="noopener noreferrer"
                          >
                            View
                          </a>
                        </div>
                      ))}
                    </div>
                  )}
                </div>
              ))}
              
              {isLoading && (
                <div className={styles.typingIndicator}>
                  Thinking...
                </div>
              )}
            </>
          )}
          <div ref={messagesEndRef} />
        </div>

        {/* Input Area */}
        <div className={styles.inputArea}>
          <textarea
            ref={inputRef}
            className={styles.messageInput}
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Ask about the book..."
            rows={1}
            disabled={isLoading}
          />
          <button
            className={styles.sendButton}
            onClick={handleSendMessage}
            disabled={!inputValue.trim() || isLoading}
            aria-label="Send message"
          >
            <span>➤</span>
          </button>
        </div>

        {/* Show error if exists */}
        {error && (
          <div style={{ padding: '10px', color: 'red', fontSize: '12px' }}>
            Error: {error}
          </div>
        )}
      </div>
    </div>
  );
};

export default ChatWidget;