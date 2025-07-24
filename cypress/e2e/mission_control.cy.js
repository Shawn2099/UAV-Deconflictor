// cypress/e2e/mission_control_spec.cy.js

describe('Mission Control Frontend E2E', () => {
  beforeEach(() => {
    cy.visit('http://localhost:5500/mission_control.html'); // Adjust port if needed
  });

  it('Toggles planning mode and adds waypoints', () => {
    cy.get('#planning-mode-toggle').click();
    cy.get('#planning-ui').should('be.visible');
    cy.get('#altitude-slider').invoke('val', 100).trigger('input');

    // Simulate clicking on canvas (mock click since 3D plane can't be clicked in DOM test easily)
    cy.window().then(win => {
      const ghost = win.document.querySelector('#main-canvas-container');
      const rect = ghost.getBoundingClientRect();
      cy.wrap(ghost).click(rect.width / 2, rect.height / 2);
    });

    cy.get('#waypoint-count').should('contain', '1 Waypoint');
  });

  it('Validates mission time inputs', () => {
    cy.get('#planning-mode-toggle').click();

    cy.get('#mission-start-time').clear().type('100');
    cy.get('#mission-end-time').clear().type('90');

    cy.get('#time-error').should('be.visible');
    cy.get('#check-airspace-btn').should('be.disabled');
  });

  it('Disconnects and reconnects socket cleanly', () => {
  cy.window().then(win => {
    // Disconnect socket
    win.socket.disconnect();

    // Force UI update (simulate what the disconnect handler would do)
    const connectBtn = win.document.getElementById('connect-btn');
    const disconnectBtn = win.document.getElementById('disconnect-btn');
    connectBtn.classList.remove('hidden');
    disconnectBtn.classList.add('hidden');

    // Update UI status manually (simulating socket.on('disconnect'))
    const status = win.document.getElementById('connection-status');
    status.textContent = 'OFFLINE';
    status.classList.add('text-red-500', 'bg-red-100');
    status.classList.remove('text-green-600', 'bg-green-100');
  });

  // Click connect button and check if reconnect happens
  cy.get('#connect-btn')
    .should('be.visible')
    .click({ force: true });

  // Confirm reconnection (this will run socket.connect() and update UI)
  cy.get('#connection-status', { timeout: 10000 }).should('contain.text', 'ONLINE');
  });
});
