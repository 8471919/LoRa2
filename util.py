
def removeGarbageInJson(payload):
    
    # to remove the garbage value
    braceByte = "}".encode()
    processed = payload.split(braceByte)
    processed.pop()
    processed.append(braceByte)
    processed = ("".encode()).join(processed)
    
    return processed
            
    