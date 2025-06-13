import React from 'react';

export default function Modal({ onClose, onBuy }) {
  return (
    <div className="modal-backdrop" onClick={onClose}>
      <div className="modal" onClick={e => e.stopPropagation()}>
        <button className="close" onClick={onClose}>×</button>
        <h2>Enjoying the vibes?</h2>
        <p>Would you like to buy me a coffee?</p>
        <button className="btn" onClick={onBuy}>☕ Buy me a coffee</button>
      </div>
    </div>
  );
}
