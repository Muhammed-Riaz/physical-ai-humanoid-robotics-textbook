import React from 'react';
import ChatWidget from '../../components/ChatWidget';

// The Root component is rendered for every page
const Root = ({ children }) => {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
};

export default Root;