# aggregator/aggregator_server.py
from flask import Flask, render_template, request, Response, abort
import requests
from urllib.parse import urljoin
from werkzeug.datastructures import Headers

app = Flask(__name__)

# Configuration
DRONE_CONFIG = {
    'scout': {
        'base_url': 'https://SCOUT_RPI_IP:5000',
        'verify_ssl': False  # Set to True if using proper SSL certificates
    },
    'delivery': {
        'base_url': 'https://DELIVERY_RPI_IP:5000',
        'verify_ssl': False  # Set to True if using proper SSL certificates
    }
}

# List of headers that should not be forwarded
EXCLUDE_HEADERS = {
    'transfer-encoding', 
    'content-encoding',
    'connection',
    'keep-alive',
    'proxy-authenticate',
    'proxy-authorization',
    'te',
    'trailer',
    'upgrade'
}

def proxy_request(drone_type, path=''):
    """Proxy requests to the appropriate drone GCS"""
    if drone_type not in DRONE_CONFIG:
        abort(404, description="Drone type not found")
    
    config = DRONE_CONFIG[drone_type]
    url = urljoin(config['base_url'], path)
    
    try:
        # Forward the request to the target GCS
        resp = requests.request(
            method=request.method,
            url=url,
            headers={k: v for k, v in request.headers if k.lower() not in EXCLUDE_HEADERS},
            data=request.get_data(),
            cookies=request.cookies,
            allow_redirects=False,
            verify=config['verify_ssl'],
            stream=True
        )
        
        # Build response headers
        headers = Headers()
        for key, value in resp.headers.items():
            if key.lower() not in EXCLUDE_HEADERS:
                headers.add(key, value)
        
        # Return the response
        return Response(
            resp.iter_content(chunk_size=1024),
            status=resp.status_code,
            headers=headers,
            content_type=resp.headers.get('content-type')
        )
    except requests.exceptions.RequestException as e:
        # Return error page if the drone GCS is unreachable
        return f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Drone GCS Unavailable</title>
            <style>
                body {{ 
                    font-family: Arial, sans-serif; 
                    text-align: center; 
                    padding: 50px; 
                    color: #333;
                }}
                .error-container {{ 
                    max-width: 600px; 
                    margin: 0 auto; 
                    padding: 20px; 
                    background: #f9f9f9; 
                    border-radius: 5px; 
                    box-shadow: 0 0 10px rgba(0,0,0,0.1);
                }}
                h1 {{ color: #d9534f; }}
                .btn {{ 
                    display: inline-block; 
                    padding: 10px 20px; 
                    background: #2c3e50; 
                    color: white; 
                    text-decoration: none; 
                    border-radius: 4px; 
                    margin-top: 20px;
                }}
                .btn:hover {{ background: #3d566e; }}
            </style>
        </head>
        <body>
            <div class="error-container">
                <h1>Drone GCS Unavailable</h1>
                <p>The {drone_type.capitalize()} Drone GCS is currently unreachable.</p>
                <p><small>Error: {str(e)}</small></p>
                <a href="/" class="btn">Return to Dashboard</a>
            </div>
        </body>
        </html>
        """, 502

# Main route
@app.route('/')
def index():
    return render_template('index.html')

# Proxy routes
# @app.route('/scout', defaults={'path': ''})
# @app.route('/scout/<path:path>')
# def scout_proxy(path):
#     return proxy_request('scout', path)

@app.route('/scout')
def scout():
    return render_template('scout.html')

# @app.route('/delivery', defaults={'path': ''})
# @app.route('/delivery/<path:path>')
# def delivery_proxy(path):
#     return proxy_request('delivery', path)

@app.route('/delivery')
def delivery():
    return render_template('delivery.html')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)