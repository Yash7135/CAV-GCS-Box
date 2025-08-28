
function update()
    local value = param:get('SCR_USER3')
    if value then
        gcs:send_text(6, string.format('LUA: SAFETYSTATE: %i',value))
    else
        gcs:send_text(6, 'FAILED TO GET SAFETY STATE')
    end
end 
    
return update()
