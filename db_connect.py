import json
import os

from pymongo.mongo_client import MongoClient
from pymongo.server_api import ServerApi
from urllib.parse import quote


class DBConfig:
    def __init__(self):
        cd = os.path.dirname(os.path.realpath(__file__)).replace("scripts", "config")
        config_full_path = os.path.join(cd, "dbconfig.json")
        file = open(config_full_path, "r")
        self.config = json.loads(file.read())


class DBConnect:
    def __init__(self):
        db_config = DBConfig()
        config = db_config.config
        self.uri = os.environ.get('MONGODB_URI')
        self.db_name = config["db_name"]
        self.collection_name_header = config["collection_name_header"]
        self.collection_name_trajectories = config["collection_name_trajectories"]
        self.collection_name_metrics = config["collection_name_metrics"]
    
    def connect(self):
        # Create a new client and connect to the server
        client = MongoClient(self.uri, server_api=ServerApi('1'))
        try:
            # Send a ping to confirm a successful connection
            client.admin.command('ping')
            print("Pinged your deployment. You successfully connected to MongoDB!")
            self.client = client
            self.db = client[self.db_name]
            self.collection_header = self.db[self.collection_name_header]
            self.collection_trajectories = self.db[self.collection_name_trajectories]
            self.collection_metrics = self.db[self.collection_name_metrics]

        except Exception as e:
            print(e)
    
    def insert_trajectories_document(self, document):
        # Insert a document into the collection
        return self.collection_trajectories.insert_one(document)
    
    def insert_header_document(self, document):
        # Insert a document into the collection
        return self.collection_header.insert_one(document)
    
    def insert_metrics_document(self, document):
        # Insert a document into the collection
        return self.collection_metrics.insert_one(document)

    def find_document_header(self, query):
        # Find documents in the collection based on a query
        return self.collection_header.find(query)
    
    def find_document_trajectories(self, query):
        # Find documents in the collection based on a query
        return self.collection_trajectories.find(query)

    def find_document_metrics(self, query):
        # Insert a document into the collection
        return self.collection_metrics.find(query)

    def update_document_metrics(self, query, update_data):
        # Update documents in the collection based on a query
        return self.collection_metrics.update_many(query, {"$set": update_data})

    def delete_document(self, query, collection):
        # Delete documents in the collection based on a query
        return self.collection.delete_many(query)
    
    def count_documents_metrics(self, query):
        # Count documents in the collection based on a query
        return self.collection_metrics.count_documents(query)

    def close_connection(self):
        # Close the MongoDB connection
        self.client.close()
