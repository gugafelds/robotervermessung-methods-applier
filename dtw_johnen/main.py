import functions_framework
from dtw_johnen import DTW_Johnen
from flask import jsonify
import json
from bson import json_util


headers = {
    'Access-Control-Allow-Origin': "*",
    'Access-Control-Allow-Methods': "*",
    'Access-Control-Allow-Headers': "*",
    'Access-Control-Max-Age': '3600',
    'Vary': 'Origin'
}

@functions_framework.http
def dtw_johnen(request):
    if request.method == 'OPTIONS':
        return '', 204, headers

    if request.method == 'POST':
        raw_data = request.get_data(as_text=True)
        try:
            data = json.loads(raw_data)
            response = DTW_Johnen(data).dtw_selective_interpolation()

        except ValueError as e:
            return f"Error parsing JSON: {e}", 400
        return jsonify(json_util.dumps(response)), 200, headers

    return "Unsupported HTTP method", 405
