document.querySelectorAll('pre').forEach(pre => {
    // Check if the <pre> element has an ancestor with the class 'highlight-output'
    if (pre.closest('.highlight-output')) {
        return; // skip
    }

    // Create a new button element
    const button = document.createElement('button');
    button.className = 'copy-button'; // assign class for styling
    button.setAttribute('aria-label', 'Copy code to clipboard'); // accessibility label for screen readers

    // Set the inner HTML of the button to include an SVG icon
    button.innerHTML = `
        <svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" class="copy-button-icon">
            <rect x="9" y="9" width="13" height="13" rx="2" ry="2"></rect>
            <path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"></path>
        </svg>
    `;
    button.title = 'copy';

    // Set up the onclick event handler for the button
    button.onclick = function () {
        docmakerCopyCode(pre, button);
    };

    // Append the button to the <pre> element
    pre.appendChild(button);
});

function docmakerCopyCode(pre, button) {
    // Extract the full text content from the <pre> element
    let code = pre.textContent;

    // Trim trailing spaces from each line and remove extra whitespace at the end
    code = code.split('\n')
        .map(line => line.replace(/\s+$/, ''))
        .join('\n')
        .trim();

    // Capture button state, to restore later
    const originalIcon = button.innerHTML;
    const originalTitle = button.title;

    // Copy to clipboard
    docmakerCopyToClipboard(code, button.ownerDocument.body)
        .then(() => {
            button.innerHTML = '&#10003;'; // Unicode check mark, for success
            button.title = 'copied';
        })
        .catch(err => {
            console.error('Error copying text: ', err);
            button.innerHTML = '&#10060;'; // Unicode cross mark, for error
            button.title = 'failed';
        })
        .finally(() => {
            // Restore the original button icon after a delay in both cases
            setTimeout(() => {
                button.innerHTML = originalIcon;
                button.title = originalTitle;
            }, 1000);
        });
}

function docmakerCopyToClipboard(text, body) {
    return new Promise((resolve, reject) => {
        // Try using the modern Clipboard API
        if (navigator.clipboard && navigator.clipboard.writeText) {
            navigator.clipboard.writeText(text)
                .then(() => {
                    console.log('Text copied to clipboard using Clipboard API');
                    resolve(); // Resolve the promise on success
                })
                .catch(err => {
                    oldCopyToClipboard(text, body, resolve, reject);
                });
        } else {
            // Fallback to execCommand if Clipboard API is not supported
            oldCopyToClipboard(text, body, resolve, reject);
        }
    });

    // Fallback method using execCommand
    function oldCopyToClipboard(text, body, resolve, reject) {
        const textArea = document.createElement('textarea');
        textArea.value = text;
        body.appendChild(textArea);
        textArea.select();
        try {
            const successful = document.execCommand('copy');
            if (successful) {
                console.log('Text copied to clipboard using execCommand');
                resolve(); // Resolve the promise on success
            } else {
                reject(new Error('Failed to copy text using execCommand'));
            }
        } catch (err) {
            console.error('Oops, unable to copy using execCommand', err);
            reject(err); // Reject the promise on error
        } finally {
            body.removeChild(textArea);
        }
    }
}