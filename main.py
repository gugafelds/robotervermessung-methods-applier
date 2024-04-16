import functions_framework
from euclidean_distance import EuclidianDistance
from flask import jsonify
import json

headers = {
    'Access-Control-Allow-Origin': '*',
    'Access-Control-Allow-Methods': 'POST, OPTIONS',
    'Access-Control-Allow-Headers': 'Content-Type',
    'Access-Control-Max-Age': '3600'
}


@functions_framework.http
def euclidean_distance(request):
    if request.method == 'OPTIONS':
        return '', 204, headers

    if request.method == 'POST':
        raw_data = request.get_data(as_text=True)
        try:
            data = json.loads(raw_data)
            response = EuclidianDistance(data).compute_distance_method()

        except ValueError as e:
            return f"Error parsing JSON: {e}", 400
        return jsonify(response), 200, headers

    return "Unsupported HTTP method", 405
