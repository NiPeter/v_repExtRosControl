simRosControl={}

function simRosControl.getNamespace()
    local namespace = sim.getScriptSimulationParameter(sim.handle_self,'namespace')

    -- in case there is no 'namespace' parameter or parameter is empty
    if (namespace == nil) or (namespace == '') then
        return ''
    end

    -- In case there is no '/' at the end of namespace
    -- the resolved namespace name would be e.g namespacebluespot_light
    -- so we must add '/' at the end of 'namespace' parameter
    -- then resolved namespace will be e.g namespace/bluespot_light
    if string.sub(namespace,-1) ~= '/' then
        namespace = namespace..'/'
    end
    -- namespace should be lowercase so
    namespace = namespace:lower()
    return namespace
end

function simRosControl.getObjectNamespace(object_handle)

    local script_handle = sim.getScriptAssociatedWithObject(object_handle)
    local namespace = sim.getScriptSimulationParameter(script_handle,'namespace')
    -- in case there is no 'namespace' parameter or parameter is empty
    if (namespace == nil) or (namespace == '') then
        return ''
    end

    -- In case there is no '/' at the end of namespace
    -- the resolved namespace name would be e.g namespacebluespot_light
    -- so we must add '/' at the end of 'namespace' parameter
    -- then resolved namespace will be e.g namespace/bluespot_light
    if string.sub(namespace,-1) ~= '/' then
        namespace = namespace..'/'
    end
    -- namespace should be lowercase so
    namespace = namespace:lower()
    return namespace
end

return simRosControl
