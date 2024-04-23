
omnibase_world: { X: [0, 0., .1] }
        
omnibase(omnibase_world): { joint: transXYPhi, shape: marker, size: [.1], color: [.0] }
omnibase_frame(omnibase): { mesh: <omnibase.ply> }
omnibase_battery(omnibase): { Q:[.12 .1 .1], shape:box, size:[.18 .13 .2], color:[1.] }

# Collision
omnibase_coll(omnibase): { shape: cylinder, color: [1.,1.,1.,.2], size: [.14, .4], noVisual, contact: -2 }
